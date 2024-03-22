/*
 * @Author: yuquan xu
 * @Date: 2023-03-17 11:06:36
 * @Last Modified by: yuquan xu
 * @Last Modified time: 2023-03-17 15:14:59
 */
#ifndef _MOTOR_LQR_CONTROL_H
#define _MOTOR_LQR_CONTROL_H
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <multi_msgs/multi_info.h>
#include <sensor_msgs/Imu.h>
namespace motor_control{
class Lqr_control {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Lqr_control(ros::NodeHandle &nh_){
        init(nh_);
    }

    ~Lqr_control(){

    }

    void init(ros::NodeHandle &nh_);
    void multi_info_callback(const multi_msgs::multi_info &info);


    void callback_imu();


    ros::NodeHandle nh;

    //订阅话题 /multi_info_sub
    ros::Subscriber multi_info_sub;





};

}










#endif