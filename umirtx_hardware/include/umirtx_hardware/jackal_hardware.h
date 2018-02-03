#ifndef UMIRTX_BASE_UMIRTX_HARDWARE_H
#define UMIRTX_BASE_UMIRTX_HARDWARE_H

#include "boost/thread.hpp"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"

namespace umirtxbase
{

class UmirtxHardware : public hardware_interface::RobotHW
{
public:
  UmirtxHardware();
  void read();
  void write();

private:
  ros::NodeHandle nh_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  // These are mutated on the controls thread only.
  struct Joint
  {
    double position;
    double position_command;
    double velocity;
    double effort;

    Joint() : position(0), velocity(0), effort(0), position_command(0)
    {
    }
  }
  joints_[7];
};

}  // namespace umirtxbase

#endif 
