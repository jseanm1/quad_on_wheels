#include <quad_on_wheels/motor_control_plugin.hpp>
#include <stdio.h>
#include <boost/bind.hpp>

namespace gazebo {
  MotorControlPlugin::MotorControlPlugin() {

  }

  void MotorControlPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr) {
    std::cout << "Loading MotorControlPlugin..." << std::endl;

    this->model = parent;

    this->linkM1 = this->model->GetLink("motor1");
    this->linkM2 = this->model->GetLink("motor2");
    this->linkM3 = this->model->GetLink("motor3");
    this->linkM4 = this->model->GetLink("motor4");

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MotorControlPlugin::onUpdate, this, _1));

    std::cout << "MotorControlPlugin loaded." << std::endl; 
  }

  void MotorControlPlugin::onUpdate(const common::UpdateInfo&) {
    gazebo::math::Vector3 force(0.0, 0.0, 10.0);

    linkM1->AddRelativeForce(force);
    linkM2->AddRelativeForce(force);
    linkM3->AddRelativeForce(force);
    linkM4->AddRelativeForce(force);
  }

  GZ_REGISTER_MODEL_PLUGIN(MotorControlPlugin)
}
