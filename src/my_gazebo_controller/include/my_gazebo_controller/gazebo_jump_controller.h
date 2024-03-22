#ifndef GAZEBO_JUMP_CONTROLLER_H_
#define GAZEBO_JUMP_CONTROLLER_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <signal.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>

#include <iostream>

#include "my_lqr_controller/leg_kinematics.h"
#include "std_msgs/String.h"

namespace Jump_controller {

class jump_controller {
 public:
  jump_controller();
  jump_controller(ros::NodeHandle &nodehandle);
  ~jump_controller();

  void receveMessage();
  void publish();
  static void MySigintHandler(int sig);  // ctrl+c 退出ros
  void init();

 private:
  ros::NodeHandle nodehandle_;
  ros::Subscriber subscriber_[7];
  ros::Publisher publisher_[7];
  float pos[8], vel[8], eff[8];
  std_msgs::Int8 sim_jump_flag;
  std_msgs::Int8 stop_ak_flag;
  float Current_Height;
  float init_y;  // 初始高度
  LegKin Leg_Calcu;
  std_msgs::Float64MultiArray msg_leg;
  int has_hight;
  void JointStateCallback(const sensor_msgs::JointStateConstPtr &msg);
  void simulation_start_jumpCallback(const std_msgs::Int8::ConstPtr &msg);
};
}  // namespace  Jump_controller

#endif