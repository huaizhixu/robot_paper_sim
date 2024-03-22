#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <time.h>

double vlinear;
int motor_forward, motor_turn_left, motor_turn, ton;
double vx, vy, vtheta_z;
std_msgs::Float64MultiArray msg_leg;

void callback(const sensor_msgs::Joy::ConstPtr &Joy)
{

    if (Joy->buttons[ton])
    {
        vx = (Joy->axes[motor_forward]) * vlinear;

        msg_leg.data.clear();
        msg_leg.data.push_back(vx);
        msg_leg.data.push_back(vx);

        ROS_INFO("linear x :%.3lf", vx);
    }
    else
    {

        if (Joy->buttons[motor_turn])
        {

            vx = (Joy->axes[motor_turn_left]) * vlinear;

            msg_leg.data.clear();
            msg_leg.data.push_back(-vx);
            msg_leg.data.push_back(vx);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy");

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    ros::Rate r(20);

    n.param<int>("motor_forward", motor_forward, 7);
    n.param<int>("motor_turn", motor_turn, 1);
    n.param<int>("motor_turn_left", motor_turn_left, 6);

    n.param<double>("vel_linear", vlinear, 0.15);
    n.param<int>("button", ton, 5);
    sub = n.subscribe<sensor_msgs::Joy>("joy", 10, callback);
    pub = n.advertise<std_msgs::Float64MultiArray>("/forw_motor_tor_mult", 1);

    while (n.ok())
    {

        std_msgs::Float64MultiArray Tor_motor;

        Tor_motor = msg_leg;

        pub.publish(Tor_motor);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
