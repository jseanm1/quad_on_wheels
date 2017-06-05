#include <quad_on_wheels/motor_control_plugin.hpp>
#include <stdio.h>
#include <boost/bind.hpp>
#include <geometry_msgs/Quaternion.h>
#include <algorithm>

namespace gazebo {
  MotorControlPlugin::MotorControlPlugin() {
    std::cout << "Initializing thrust and torque values" << std::endl;
    this->initTnQ();

    std::cout << "Initializing gains" << std::endl;
    this->initGains();
  }

  void MotorControlPlugin::initTnQ() {
    // initialize thrusts and torques to zero
    boost::mutex::scoped_lock(this->tqMutex);
    this->tM1 = 0.0;
    this->tM2 = 0.0;
    this->tM3 = 0.0;
    this->tM4 = 0.0;
    this->qM1 = 0.0;
    this->qM2 = 0.0;
    this->qM3 = 0.0;
    this->qM4 = 0.0;
  }

  void MotorControlPlugin::initGains() {
    // initialize gains
    boost::mutex::scoped_lock(this->gainMutex);
    this->Kvx_p = 0.01;
    this->Kp_p = 0.01;
    this->Kp_d = 0.01;
    this->Ky_p = 0.01;
    this->W = 0.0;
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

    // Advertise Services
    this->updateGainsServer = nh.advertiseService("/updateGains", &MotorControlPlugin::updateGains, this);

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
    // Access gz resources: START
    this->gzMutex.lock();

    this->gzPose = this->linkBaseLink->GetWorldPose();
    this->gzLinVel = this->linkBaseLink->GetWorldLinearVel();
    this->gzLinAcc = this->linkBaseLink->GetWorldLinearAccel();
    this->gzAngVel = this->linkBaseLink->GetWorldAngularVel();
    this->gzAngAcc = this->linkBaseLink->GetWorldAngularAccel();

    // Access gz resources : END
    this->gzMutex.unlock();

    // Access thrust and torque values : START
    this->tqMutex.lock();

    gazebo::math::Vector3 tVM1(0.0, 0.0, this->tM1);
    gazebo::math::Vector3 tVM2(0.0, 0.0, this->tM2);
    gazebo::math::Vector3 tVM3(0.0, 0.0, this->tM3);
    gazebo::math::Vector3 tVM4(0.0, 0.0, this->tM4);

    gazebo::math::Vector3 qVM1(0.0, 0.0, this->qM1);
    gazebo::math::Vector3 qVM2(0.0, 0.0, this->qM2);
    gazebo::math::Vector3 qVM3(0.0, 0.0, this->qM3);
    gazebo::math::Vector3 qVM4(0.0, 0.0, this->qM4);

    // Access thrust and torque values : END
    this->tqMutex.unlock();

    // Apply forces and torques to the model
    this->linkM1->AddRelativeForce(tVM1);
    this->linkM2->AddRelativeForce(tVM2);
    this->linkM3->AddRelativeForce(tVM3);
    this->linkM4->AddRelativeForce(tVM4);

    this->linkM1->AddRelativeTorque(qVM1);
    this->linkM2->AddRelativeTorque(qVM2);
    this->linkM3->AddRelativeTorque(qVM3);
    this->linkM4->AddRelativeTorque(qVM4);
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
    float vp = this->gzAngVel.y;
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

    //std::cout << "r: " << r << ", p: " << p << ", y: " << y << std::endl;

    // Access gains : START
    this->gainMutex.lock();

    float kvx_p = this->Kvx_p;
    float kp_p = this->Kp_p;
    float kp_d = this->Kp_d;
    float ky_p = this->Ky_p;
    float w = this->W;

    // Access gains : END
    this->gainMutex.unlock();

    float vx_d = 5.0;
    float p_d;
    float p_max = 1.306;
    float dwp;
    float dwy;

    double thrustM1, thrustM2, thrustM3, thrustM4;
    double torqueM1, torqueM2, torqueM3, torqueM4;
    double rpsM1, rpsM2, rpsM3, rpsM4;

    // Velocity controller
    p_d = std::min(kvx_p * (vx_d - vx), p_max);
    dwp = kp_p * ((p_d - p) - kp_d * vp);
    // std::cout << "X: " << this->gzLinVel.x << ", y:" << this->gzLinVel.y << ", z: " << this->gzLinVel.z << std::endl;
    // std::cout << "pd: " << p_d << ", dwp: " << dwp << std::endl;
    // Yaw controller
    dwy = ky_p * (y);
    // dwy = 0; // Disable yaw controller

    rpsM1 = w - dwp + dwy;
    rpsM2 = w - dwp - dwy;
    rpsM3 = w + dwp + dwy;
    rpsM4 = w + dwp - dwy;

    this->propellerSim.getThrustAndTorque(thrustM1, torqueM1, rpsM1);
    this->propellerSim.getThrustAndTorque(thrustM2, torqueM2, rpsM2);
    this->propellerSim.getThrustAndTorque(thrustM3, torqueM3, rpsM3);
    this->propellerSim.getThrustAndTorque(thrustM4, torqueM4, rpsM4);

    std::cout << thrustM1 << "," << torqueM1 << "," << rpsM1 << ","
      << thrustM2 << "," << torqueM2 << "," << rpsM2 << ","
      << thrustM3 << "," << torqueM3 << "," << rpsM3 << ","
      << thrustM4 << "," << torqueM4 << "," << rpsM4 << std::endl;

    // Update new thrust and torque values
    // Access thrust and torque values : START
    this->tqMutex.lock();

    this->tM1 = thrustM1;
    this->tM2 = thrustM2;
    this->tM3 = thrustM3;
    this->tM4 = thrustM4;

    this->qM1 = -torqueM1; // Clockwise rotation
    this->qM2 = torqueM2;
    this->qM3 = -torqueM3; // Clockwise rotation
    this->qM4 = torqueM4;

    // Access thrust and torque values : END
    this->tqMutex.unlock();
  }

  bool MotorControlPlugin::updateGains(quad_on_wheels::UpdateGains::Request &req, quad_on_wheels::UpdateGains::Response &res) {
    boost::mutex::scoped_lock(this->gainMutex);
    this->Kvx_p = req.kvx_p;
    this->Kp_p = req.kp_p;
    this->Kp_d = req.kp_d;
    this->Ky_p = req.ky_p;
    this->W = req.w;

    //std::cout << "Kvx: " << this->Kvx << ", Kp: " << this->Kp << ", Ky: " << this->Ky << ", W: " << this->W << std::endl;

    res.result = true;
    return true;
  }

  GZ_REGISTER_MODEL_PLUGIN(MotorControlPlugin)
}
