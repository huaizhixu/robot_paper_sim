#include "my_wheel_controller/controller.h"
#include <geometry_msgs/Vector3.h>
#include <multi_msgs/ak80_info.h>
#include <multi_msgs/multi_info.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include <cmath>
#define PI 3.1415926
const double pi = acos(-1.0);

float x;
float dx;
float theta;
float dtheta;
float delta_yaw;
float ddelta_yaw;

std_msgs::Float64 msg13;
std_msgs::Float64 msg14;

geometry_msgs::Vector3 my_pid_out_info;

float out_motor;

Controller outer;

void set_Motor(float tor)
{
    msg13.data = tor;
    msg14.data = tor;
}

void multinfo_Callback(const multi_msgs::multi_info::ConstPtr &info)
{

    x = info->x;
    dx = info->dx;
    theta = info->theta;
    dtheta = info->dtheta;
    delta_yaw = info->delta_yaw;
    ddelta_yaw = info->ddelta_yaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    // 订阅多传感器信息
    ros::Subscriber multi_info = nh.subscribe("/multi_info_pub", 1000, multinfo_Callback);
    // 发布力矩
    ros::Publisher left_motor_wheel_id13 = nh.advertise<std_msgs::Float64>("motor_wheel_id13", 1000);
    ros::Publisher right_motor_wheel_id14 = nh.advertise<std_msgs::Float64>("motor_wheel_id14", 1000);

    ros::Publisher mypid = nh.advertise<geometry_msgs::Vector3>("/my_pid_out", 1000);

    outer.maxOutput = 10.0;
    outer.maxIntegral = 0.1; // 0.1
    outer.setConstants(12.5, 0, 1);

    //   outer.setConstants(0, 0, 0.3);

    ros::Rate loop_rate(500); // 100HZ
    while (ros::ok())
    {
        my_pid_out_info.z = dtheta;
        my_pid_out_info.y = theta;

        outer.compute(0, theta, dtheta);
        out_motor = outer.getOutput();

        set_Motor(out_motor);

        my_pid_out_info.x = out_motor;

        left_motor_wheel_id13.publish(msg13);
        right_motor_wheel_id14.publish(msg14);
        mypid.publish(my_pid_out_info);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}