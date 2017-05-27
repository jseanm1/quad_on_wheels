#include <quad_on_wheels/motor_control_plugin.hpp>
#include <stdio.h>
#include <boost/bind.hpp>
#include <geometry_msgs/Quaternion.h>

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

    float k1 = 10;
    float k2 = 1.0;
    float f3 = k1*(1.309 - p) + 1;
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
    q.w = this->gzPose.rot.w;
    q.x = this->gzPose.rot.x;
    q.y = this->gzPose.rot.y;
    q.z = this->gzPose.rot.z;

    // Access gz resources: END
    this->gzMutex.unlock();

    float thetaD = 0.523598776; // 30 degrees
    tf::Quaternion tfQ;
    tf::quaternionMsgToTF(q, tfQ);
    double r,p,y;
    tf::Matrix3x3 rot(tfQ);
    rot.getRPY(r,p,y);
  }

  GZ_REGISTER_MODEL_PLUGIN(MotorControlPlugin)
}
