#include <quad_on_wheels/motor_control_plugin.hpp>
#include <stdio.h>
#include <boost/bind.hpp>

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

    this->model = parent;

    this->linkM1 = this->model->GetLink("motor1");
    this->linkM2 = this->model->GetLink("motor2");
    this->linkM3 = this->model->GetLink("motor3");
    this->linkM4 = this->model->GetLink("motor4");

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MotorControlPlugin::onUpdate, this, _1));

    std::cout << "MotorControlPlugin loaded." << std::endl; 
  }

  void MotorControlPlugin::onUpdate(const common::UpdateInfo&) {
    // gazebo::math::Vector3 force(0.0, 0.0, 10.0);

    // linkM1->AddRelativeForce(force);
    // linkM2->AddRelativeForce(force);
    // linkM3->AddRelativeForce(force);
    // linkM4->AddRelativeForce(force);
  }

  void MotorControlPlugin::simpleControllerCb(geometry_msgs::Wrench::ConstPtr wrench) {
    std::cout << "Msg recieved" << std::endl;
  }

  GZ_REGISTER_MODEL_PLUGIN(MotorControlPlugin)
}
