#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <string>
#include <time.h>

double vlinear, vangular;
int motor_forward, motor_back, motor_turn, ton;
double vx, vy, vtheta_z;

void callback(const sensor_msgs::Joy::ConstPtr &Joy)
{
    std_msgs::Float64 v;

    if (Joy->buttons[ton] != 0)
    {
        vx = (Joy->axes[motor_forward]) * vlinear;
        ROS_INFO("linear x :%.3lf", vx);
        //  pub.publish(v);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy");

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    ros::Rate r(100);

    n.param<int>("motor_forward", motor_forward, 7);
    n.param<int>("motor_back", motor_back, 6);
    n.param<int>("motor_turn", motor_turn, 3);
    n.param<double>("vel_linear", vlinear, 0.6);
    n.param<int>("button", ton, 5);
    sub = n.subscribe<sensor_msgs::Joy>("joy", 10, callback);
    pub = n.advertise<std_msgs::Float64>("/forw_motor_tor", 5);

    while (n.ok())
    {

        std_msgs::Float64 Tor_motor;

        // if(Joy->buttons[ton])
        //     {
        Tor_motor.data = vx;

        // ROS_INFO("linear x y:%.3lf %.3lf  angular:%.3lf",v.linear.x,v.linear.y,v.angular.z);
        pub.publish(Tor_motor);
        // }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
