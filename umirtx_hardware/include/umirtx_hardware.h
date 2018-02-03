#ifndef UMIRTX_BASE_UMIRTX_HARDWARE_H
#define UMIRTX_BASE_UMIRTX_HARDWARE_H

#include "boost/thread.hpp"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "ros/publication.h"
#include "ros/subscribe_options.h"
#include "sensor_msgs/JointState.h"
#include <sys/time.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#define BUFLEN 200
#define NPACK 1
#define PORT 5000

namespace umirtxbase
{

class UmirtxHardware : public hardware_interface::RobotHW
{
public:

  UmirtxHardware();
  struct timeval timevalue;
  void read();
  void write();
    void feedbackCallback(const  sensor_msgs::JointState::ConstPtr& msg);
    void UmirtxUDPclient(const char * umirtx_ip);
    bool connect_ok;
    const char * umirtx_ip;
private:
  ros::NodeHandle nh_;
    ros::Subscriber feedback_sub_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  // These are mutated on the controls thread only.
struct packet  //alap UDP csomag
{
    int32_t pos;
    int32_t cmd;
    int16_t vel;
    int16_t eff;
    packet(): pos(0),cmd(0),vel(0),eff(0)
    {

    }
}packs[8];

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
  joints_[8];

  void umiconverter(packet* P , Joint* J);
  void pcconverter(packet* P,Joint* J);
  sensor_msgs::JointState::ConstPtr feedback_msg_;

 boost::mutex feedback_msg_mutex_;
};

}  // namespace umirtxbase

#endif 
