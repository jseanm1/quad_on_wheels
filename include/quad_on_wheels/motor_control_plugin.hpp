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
      physics::LinkPtr linkM1, linkM2, linkM3, linkM4;
      event::ConnectionPtr updateConnection;
  };
}