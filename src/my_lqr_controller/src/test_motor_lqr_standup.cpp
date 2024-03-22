/*
 * @Author: yuquan xu
 * @Date: 2023-03-17 10:50:25
 * @Last Modified by: yuquan xu
 * @Last Modified time: 2023-03-17 14:48:00
 */
//控制实体机器人的LQR实现

#include"my_lqr_controller/motor_lqr_control.h"
#include "ros/ros.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motor_control");
    ros::NodeHandle nh;

    motor_control::Lqr_control motor_control=motor_control::Lqr_control(nh);

    ros::Rate loop_rate(500);
    ros::spin();
    loop_rate.sleep();
    return 0;
}