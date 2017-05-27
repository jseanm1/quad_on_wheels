#include <quad_on_wheels/motor_control_plugin.hpp>
#include <stdio.h>
#include <boost/bind.hpp>
#include <geometry_msgs/Quaternion.h>
#include <algorithm>

namespace gazebo {
  MotorControlPlugin::MotorControlPlugin() {

  }

  void MotorControlPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr) {
    if (!ros::isInitialized()) {
      std::cout << "ROS is not initialized" << std::endl;

      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "motor_control_plugin");
    }
    std::cout << "ROS is initialized" << std::endl;
    std::cout << "Loading MotorControlPlugin..." << std::endl;

    // Setup subscribers
    ros::NodeHandle nh;
    this->simpleControllerSub = nh.subscribe("/controller/simple", 10, &MotorControlPlugin::simpleControllerCb, this);

    this->velocityController1Sub = nh.subscribe("/controller/velocity/1", 10, &MotorControlPlugin::velocityController1Cb, this);

    this->model = parent;
    this->linkBaseLink = this->model->GetLink("base_link");
    this->linkM1 = this->model->GetLink("motor1");
    this->linkM2 = this->model->GetLink("motor2");
    this->linkM3 = this->model->GetLink("motor3");
    this->linkM4 = this->model->GetLink("motor4");

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MotorControlPlugin::onUpdate, this, _1));

    std::cout << "MotorControlPlugin loaded." << std::endl; 
  }

  void MotorControlPlugin::onUpdate(const common::UpdateInfo&) {
    // Update state of the base link
    boost::mutex::scoped_lock(this->gzMutex);
    this->gzPose = this->linkBaseLink->GetWorldPose();
    this->gzLinVel = this->linkBaseLink->GetWorldLinearVel();
    this->gzLinAcc = this->linkBaseLink->GetWorldLinearAccel();
    this->gzAngVel = this->linkBaseLink->GetWorldAngularVel();
    this->gzAngAcc = this->linkBaseLink->GetWorldAngularAccel();
  }

  void MotorControlPlugin::simpleControllerCb(geometry_msgs::Wrench::ConstPtr wrench) {
    gazebo::math::Vector3 force(wrench->force.x, wrench->force.y,wrench->force.z);
    gazebo::math::Vector3 torque(wrench->torque.x, wrench->torque.y, wrench->torque.z);

    // Access gz resources: START
    this->gzMutex.lock();
    geometry_msgs::Quaternion q;
    q.w = this->gzPose.rot.w;
    q.x = this->gzPose.rot.x;
    q.y = this->gzPose.rot.y;
    q.z = this->gzPose.rot.z;

    // Access gz resources: END
    this->gzMutex.unlock();

    tf::Quaternion tfQ;
    tf::quaternionMsgToTF(q, tfQ);
    double r,p,y;
    tf::Matrix3x3 rot(tfQ);
    rot.getRPY(r,p,y);

    float k1 = 0.01;
    float k2 = 0.01;
    float f3 = k1*(1.309 - p) + 0.01;
    float t3 = k2*(-y);

    gazebo::math::Vector3 forceF3(wrench->force.x, wrench->force.y,f3);
    gazebo::math::Vector3 torqueF3(wrench->torque.x, wrench->torque.y, t3);

    this->linkM1->AddRelativeForce(force);
    this->linkM2->AddRelativeForce(force);
    this->linkM3->AddRelativeForce(forceF3);
    this->linkM3->AddRelativeTorque(torqueF3);
    this->linkM4->AddRelativeForce(forceF3);
    this->linkM4->AddRelativeTorque(torqueF3);
  }

  void MotorControlPlugin::velocityController1Cb(std_msgs::Empty::ConstPtr msg) {
    // Access gz resources: START
    this->gzMutex.lock();
    geometry_msgs::Quaternion q;
    float vx = this->gzLinVel.x;
    q.w = this->gzPose.rot.w;
    q.x = this->gzPose.rot.x;
    q.y = this->gzPose.rot.y;
    q.z = this->gzPose.rot.z;

    // Access gz resources: END
    this->gzMutex.unlock();

    tf::Quaternion tfQ;
    tf::quaternionMsgToTF(q, tfQ);
    double r,p,y;
    tf::Matrix3x3 rot(tfQ);
    rot.getRPY(r,p,y);

    float vx_d = 1.0;
    float kvx = 1.0;
    float p_d;
    float p_max = 1.306;
    float kp;
    float w = 100;
    float dwp;
    float ky;
    float dwy;

    double *thrustM1, *thrustM2, *thrustM3, *thrustM4;
    double *torqueM1, *torqueM2, *torqueM3, *torqueM4;
    double rpsM1, rpsM2, rpsM3, rpsM4;

    // Velocity controller
    p_d = std::max(kvx * (vx_d - vx), p_max);
    dwp = kp * (p_d - p);

    // Yaw controller
    dwy = ky * (-y);

    rpsM1 = w - dwp + dwy;
    rpsM2 = w - dwp - dwy;
    rpsM3 = w + dwp + dwy;
    rpsM4 = w + dwp - dwy;

    this->propellerSim.getThrustAndTorque(thrustM1, torqueM1, rpsM1);
    this->propellerSim.getThrustAndTorque(thrustM2, torqueM2, rpsM2);
    this->propellerSim.getThrustAndTorque(thrustM3, torqueM3, rpsM3);
    this->propellerSim.getThrustAndTorque(thrustM4, torqueM4, rpsM4);

    gazebo::math::Vector3 fvM1(0.0, 0.0, *thrustM1);
    this->linkM1->AddRelativeForce(fvM1);
    gazebo::math::Vector3 fvM2(0.0, 0.0, *thrustM2);
    this->linkM2->AddRelativeForce(fvM2);
    gazebo::math::Vector3 fvM3(0.0, 0.0, *thrustM3);
    this->linkM3->AddRelativeForce(fvM3);
    gazebo::math::Vector3 fvM4(0.0, 0.0, *thrustM4);
    this->linkM4->AddRelativeForce(fvM4);

    gazebo::math::Vector3 tvM1(0.0, 0.0, -*torqueM1);
    this->linkM1->AddRelativeTorque(tvM1);
    gazebo::math::Vector3 tvM2(0.0, 0.0, *torqueM2);
    this->linkM2->AddRelativeTorque(tvM2);
    gazebo::math::Vector3 tvM3(0.0, 0.0, -*torqueM3);
    this->linkM3->AddRelativeTorque(tvM3);
    gazebo::math::Vector3 tvM4(0.0, 0.0, *torqueM4);
    this->linkM4->AddRelativeTorque(tvM4);
  }

  GZ_REGISTER_MODEL_PLUGIN(MotorControlPlugin)
}
