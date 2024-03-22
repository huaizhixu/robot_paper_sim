#ifndef _LQR_CONTROL_H
#define _LQR_CONTROLH
#include "std_msgs/Int32.h"
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <multi_msgs/ak80_info.h>
#include <multi_msgs/multi_info.h>
#include <multi_msgs/target_info.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

class lqr_control
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    lqr_control(ros::NodeHandle &nh)
    {
        init(nh);
    }
    ~lqr_control()
    {
    }
    void multinfo_Callback(const multi_msgs::multi_info::ConstPtr &info);
    void target_Callback(const multi_msgs::target_info::ConstPtr &info);
    void init(ros::NodeHandle &nh);

    void stand_up_lqr_Callback(const std_msgs::Int32::ConstPtr &msg);

    ros::Subscriber sub_imu;
    ros::Subscriber sub_ak80;
    ros::Subscriber multi_info;
    // stand_up_lqr站立保持平衡
    ros::Subscriber sub_stand_up_lqr;

    ros::Publisher left_motor_wheel_id13;
    ros::Publisher right_motor_wheel_id14;
    ros::Publisher state1_pub;
    std_msgs::Float64 msg13;
    std_msgs::Float64 msg14;

    // 站立标志
    int stand_up_lqr_flag;

    geometry_msgs::Vector3 my_state1_info;

    //! 计算开始的时间
    double begin;

    // 订阅目标值
    ros::Subscriber sub_target;
};

#endif