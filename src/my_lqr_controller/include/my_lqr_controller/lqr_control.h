#ifndef _LQR_CONTROL_H
#define _LQR_CONTROL_H
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

namespace my_control{
class Lqr_control {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Lqr_control(ros::NodeHandle &nh){
        init(nh);
    }
    ~Lqr_control(){}


    void init(ros::NodeHandle &nh);
    void callback_reference_position(const std_msgs::Float64::ConstPtr& msg);
    void callback_odom(const nav_msgs::Odometry::ConstPtr& msg);
    void callback_imu(const sensor_msgs::Imu::ConstPtr& msg);

    ros::Publisher left_motor_wheel;
    ros::Publisher right_motor_wheel;
    ros::Publisher left_and_right_motor_wheel;
    ros::Subscriber odometry_subscriber;
    ros::Subscriber imu_subscriber;
    ros::Subscriber reference_position_topic;
    float current_time;
    float dt;
    float error;
    float ref;
    float integral_error = 0.0;
    float current_posx;
    float current_velx;
    float current_pitch_rate;
    tf::Quaternion my_quaternion;
    double roll, pitch, yaw;
    float Yaw;
    float current_yaw_rate;
    float Pitch;
    float rot_integral_error = 0.0;
    float rot_error;
    std_msgs::Float64 left_torque;
    std_msgs::Float64 right_torque;
    std_msgs::Float64MultiArray msg_ball_wheel;

};

}

namespace Pid_control{
    class pid_control {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    pid_control(ros::NodeHandle &nh){
        init(nh);
    }
    ~pid_control(){}


    void init(ros::NodeHandle &nh);
    void callback_reference_position(const std_msgs::Float64::ConstPtr& msg);
    void callback_odom(const nav_msgs::Odometry::ConstPtr& msg);
    void callback_imu(const sensor_msgs::Imu::ConstPtr& msg);


    ros::Publisher left_motor_wheel;
    ros::Publisher right_motor_wheel;
    ros::Subscriber odometry_subscriber;
    ros::Subscriber imu_subscriber;
    ros::Subscriber reference_position_topic;
    float current_time;
    float dt;
    float error;
    float previous_error;
    float ref;
    float integral_error = 0.0;
    float current_posx;
    float current_velx;
    float current_pitch_rate;
    float current_pitch_rate_pid;

    tf::Quaternion my_quaternion;
    double roll, pitch, yaw;
    float Yaw;
    float current_yaw_rate;
    float Pitch;
    float rot_integral_error = 0.0;
    float rot_error;
    float Pid_out;
    float Pid_yaw_out;
    float Pid_pitch_out;

    std_msgs::Float64 left_torque;
    std_msgs::Float64 right_torque;
    std_msgs::Float64MultiArray msg_ball_wheel;
    //ros::NodeHandle nh;


};

}


#endif