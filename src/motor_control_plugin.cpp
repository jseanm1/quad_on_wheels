#include <quad_on_wheels/motor_control_plugin.hpp>
#include <stdio.h>
#include <boost/bind.hpp>

namespace gazebo {
  MotorControlPlugin::MotorControlPlugin() {

  }

  void MotorControlPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr) {
    printf("\n\n\n\n-------\n\n\n\n");
    std::cout << "Loading MotorControlPlugin..." << std::endl;

    this->model = parent;

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MotorControlPlugin::onUpdate, this, _1));

    std::cout << "MotorControlPlugin loaded." << std::endl; 
    std::cout << "Plugin loaded" << std::endl;
    printf("\n\n\n\n-------\n\n\n\n");
  }

  void onUpdate(const common::UpdateInfo &) {
    std::cout << ".";
  }
  GZ_REGISTER_MODEL_PLUGIN(MotorControlPlugin)
}
