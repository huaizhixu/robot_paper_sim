/*
 * @Author: yuquan xu
 * @Date: 2023-03-17 11:06:48
 * @Last Modified by: yuquan xu
 * @Last Modified time: 2023-03-17 15:52:27
 */
#include "my_lqr_controller/motor_lqr_control.h"


namespace motor_control{

void Lqr_control::init(ros::NodeHandle &nh_){


    nh=nh_;
    multi_info_sub=nh.subscribe("/multi_info_sub",5,&Lqr_control::multi_info_callback,this);
    ROS_INFO_STREAM_ONCE("Motor_control init success.....");


    //每两次输出一次log
    //ROS_INFO_STREAM_THROTTLE( 2, "INFO throttle message." );


}


void Lqr_control::multi_info_callback(const multi_msgs::multi_info &info){
        ROS_INFO("hello,callback_multi_info_callback ...");


}



void Lqr_control::callback_imu(){

        ROS_INFO("hello,callback_imu ...");
}

}

