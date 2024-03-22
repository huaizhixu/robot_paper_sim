/*
 * @Author: yuquan xu
 * @Date: 2023-06-16 19:28:30
 * @Last Modified by: yuquan xu
 * @Last Modified time: 2023-06-16 19:31:15
 */
// 球的控制运动程序，实现前进和转向
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

#include <multi_msgs/target_info.h>

std_msgs::Float64 motor13_tor;
std_msgs::Float64 motor14_tor;
std_msgs::Float64 motor_vel;
// 发布的力矩
// float motor13_tor;
// float motor14_tor;

// 球形没有速度保护
std_msgs::Int8 flag_vel_protection;

void callback(const sensor_msgs::Joy::ConstPtr &Joy)
{
    std_msgs::Float64 v;

    if (Joy->buttons[5] != 0)
    {

        if (Joy->axes[1] != 0)
        {
            motor13_tor.data = Joy->axes[1] * 1.5;
            motor14_tor.data = Joy->axes[1] * 1.5;
            motor_vel.data = Joy->axes[1] * 5;
        }

        if (Joy->axes[0] != 0)
        {
            motor13_tor.data = -Joy->axes[0] * 1.5;
            motor14_tor.data = Joy->axes[0] * 1.5;
            motor_vel.data = -Joy->axes[0] * 5;
        }
    }
    else
    {
        motor13_tor.data = 0;
        motor14_tor.data = 0;
        motor_vel.data = 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_jump_control");

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Publisher pub3;
    ros::Publisher pub4;

    sub = n.subscribe<sensor_msgs::Joy>("joy", 10, callback);
    pub = n.advertise<std_msgs::Float64>("motor_wheel_id13", 100);
    pub2 = n.advertise<std_msgs::Float64>("motor_wheel_id14", 100);
    pub3 = n.advertise<std_msgs::Int8>("vel_protection", 100);
    pub4 = n.advertise<std_msgs::Float64>("motor_wheel_vel", 100);
    flag_vel_protection.data = 0;

    ros::Rate loop_rate(150);

    while (ros::ok())
    {
        flag_vel_protection.data = 1;
        pub.publish(motor13_tor);
        pub2.publish(motor14_tor);
        pub4.publish(motor_vel);
        pub3.publish(flag_vel_protection);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}