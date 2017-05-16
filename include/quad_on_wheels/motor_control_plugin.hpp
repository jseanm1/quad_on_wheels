#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo {
  class MotorControlPlugin : public ModelPlugin {
    public:
      MotorControlPlugin();
      void Load(physics::ModelPtr, sdf::ElementPtr);
      void onUpdate(const common::UpdateInfo&);

    private:
      physics::ModelPtr model;
      event::ConnectionPtr updateConnection;
  };
}