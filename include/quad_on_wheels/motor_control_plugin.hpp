#ifndef QUAD_ON_WHEELS_NOTOR_CONTROL_PLUGIN_H
#define QUAD_ON_WHEELS_NOTOR_CONTROL_PLUGIN_H

#include <quad_on_wheels/propeller_sim.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <boost/thread/mutex.hpp>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Transform.h"
#include "tf/tf.h"

namespace gazebo {
  class MotorControlPlugin : public ModelPlugin {
    public:
      MotorControlPlugin();
      void Load(physics::ModelPtr, sdf::ElementPtr);
      void onUpdate(const common::UpdateInfo&);

    private:
      physics::ModelPtr model;
      physics::LinkPtr linkM1, linkM2, linkM3, linkM4, linkBaseLink;
      gazebo::math::Pose gzPose;
      gazebo::math::Vector3 gzLinVel, gzLinAcc, gzAngVel, gzAngAcc;

      boost::mutex gzMutex;
      
      event::ConnectionPtr updateConnection;

      ros::Subscriber simpleControllerSub;

      void simpleControllerCb(geometry_msgs::Wrench::ConstPtr);
  };
}

#endif
