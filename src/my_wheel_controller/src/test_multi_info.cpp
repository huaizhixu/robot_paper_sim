/*
 * @Author: yuquan xu
 * @Date: 2023-02-09 16:03:12
 * @Last Modified by: mikey.zhaopeng
 * @Last Modified time: 2023-02-10 14:50:02
 */
#include <multi_msgs/multi_info.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "msg_talker");
  ros::NodeHandle n;
  ros::Publisher msg_pub = n.advertise<multi_msgs::multi_info>("test_msg", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    multi_msgs::multi_info msg;

    msg.x = 0.7;
    msg.dx = 0;
    msg.theta = 0.2;
    msg.dtheta = 0.3;
    msg.delta_yaw = 0.4;
    msg.ddelta_yaw = 0.5;
    std::cout << "msg->dx=" << msg.dx << std::endl;
    std::cout << "msg->the=" << msg.theta << std::endl;
    std::cout << std::endl;

    msg_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
