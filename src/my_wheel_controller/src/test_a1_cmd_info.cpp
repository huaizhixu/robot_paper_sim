/*
 * @Author: yuquan xu
 * @Date: 2023-02-20 16:20:03
 * @Last Modified by: yuquan xu
 * @Last Modified time: 2023-02-22 09:49:29
 */
/*测试自己定义的电机的命令.msg文件的测试文件  */
#include <multi_msgs/a1_motor_Cmd.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "msg_talker");
  ros::NodeHandle n;
  ros::Publisher msg_pub = n.advertise<multi_msgs::a1_motor_Cmd>("a1_motor_Cmd", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    multi_msgs::a1_motor_Cmd msg;

    msg.mode = 5;
    msg.q = 1;
    msg.dq = 0.2;
    msg.tau = 0.3;
    msg.Kp = 0.4;
    msg.Kd = 0.5;
    std::cout << "msg->tau=" << msg.tau << std::endl;
    std::cout << "msg->Kd=" << msg.Kd << std::endl;
    std::cout << std::endl;

    msg_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
