/*
 * @Author: yuquan xu
 * @Date: 2023-02-10 10:43:52
 * @Last Modified by: yuquan xu
 * @Last Modified time: 2023-03-20 15:01:45
 */
#include "my_wheel_controller/lqr_control.h"
#include <multi_msgs/ak80_info.h>
#include <multi_msgs/multi_info.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lqr_control");

  ros::NodeHandle nh;

  lqr_control lqr(nh);

  ros::Rate loop_rate(500);
  ros::spin();
  loop_rate.sleep();
  //  lqr.pub();

  return 0;
}