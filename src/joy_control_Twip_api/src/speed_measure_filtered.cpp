/*
 * @Author: yuquan xu
 * @Date: 2023-02-10 10:43:52
 * @Last Modified by: yuquan xu
 * @Last Modified time: 2023-03-20 15:01:45
 */

// 多传感器，将多个信号订阅到一个节点并发布
#include <geometry_msgs/Vector3.h>
#include <multi_msgs/ak80_info.h>
#include <multi_msgs/multi_info.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

class Speed_filter
{
public:
    Speed_filter() : n_("~"), sum_speed(0.0), count(0), sum_acc_speed(0), sum_acc_acc_speed(0)
    {
        acc_speed = 0.0;
        acc_acc_speed = 0.0;
        // 先前的
        previous_speed = 0.0;
        previous_acc_speed = 0.0;
        before_time_ = 0.0;
        previous_acc_acc_speed = 0.0;
        current_speed = 0.0;

        filter_pub = n_.advertise<geometry_msgs::Vector3>("/filter_speed", 1000);
        state1_pub = n_.advertise<std_msgs::Float32>("/acc_acc_original_info", 1000);
        // 订阅电机的状态 话题为：motor_ak80_info
        sub_ak80 = n_.subscribe("/speed_measure_pub", 1000, &Speed_filter::ak80_Callback, this);
    }

    void ak80_Callback(const std_msgs::Float32::ConstPtr &motor_ak80_info)
    {

        ROS_INFO_STREAM_THROTTLE(7, "\033[45;37m Speed_filter callback \033[0m");

        double now = ros::Time::now().toSec();
        std::cout << "time:" << now - before_time_ << std::endl;
        geometry_msgs::Vector3 filter_speed;
        current_speed = motor_ak80_info->data;
        // 对速度进行滤波
        std_msgs::Float32 filter_vel;
        float B = LopPassFilter_RC_1st_Factor_Cal(0.002, 45);
        std::cout << "B:" << LopPassFilter_RC_1st_Factor_Cal(0.002, 45) << std::endl;
        filter_vel.data = LopPassFilter_RC_1st(previous_speed, current_speed, B);
        // float result = B*motor_ak80_info->data + (1-B)*previous_speed;
        previous_speed = filter_vel.data;

        // 加速度计算
        acc_speed = (filter_vel.data - previous_filter_vel) / 0.003;
        previous_filter_vel = filter_vel.data;
        filter_speed.x = motor_ak80_info->data;
        filter_speed.y = acc_speed;
        // 加加速度计算
        acc_acc_speed = (acc_speed - previous_acc_speed) / 0.003;
        previous_acc_speed = acc_speed;
        if (acc_acc_speed > 0 && acc_acc_speed - 45 > 0)
        {
            acc_acc_speed = acc_acc_speed - 45;
        }
        if (acc_acc_speed < 0 && acc_acc_speed + 45 < 0)
        {
            acc_acc_speed = acc_acc_speed + 45;
        }
        if (acc_acc_speed > 600)
        {
            acc_acc_speed += 200;
        }
        if (acc_acc_speed < -600)
        {
            acc_acc_speed -= 200;
        }
        filter_speed.z = acc_acc_speed;
        // 滤波处理
        // std_msgs::Float32 acc_acc_info;
        // acc_acc_info.data = current_speed;
        // float B = LopPassFilter_RC_1st_Factor_Cal(0.002, 10000);
        // std::cout << "B:" << LopPassFilter_RC_1st_Factor_Cal(0.002, 10000) << std::endl;
        // acc_acc_info.data = LopPassFilter_RC_1st(previous_acc_acc_speed, acc_acc_speed, B);
        state1_pub.publish(filter_vel);
        filter_pub.publish(filter_speed);

        // std_msgs::Float32 filter_acc;
        // acc_acc_speed = (speed_filter_info->data - previous_speed) /0.002;

        // LopPassFilter_RC_1st_Factor_Cal(now - before_time_, 10);
        // float B=LopPassFilter_RC_1st_Factor_Cal(0.002, 50);
        // std::cout << "B:" << LopPassFilter_RC_1st_Factor_Cal(0.002, 50)<< std::endl;

        // filter_acc.data = LopPassFilter_RC_1st(previous_speed, speed_filter_info->data, B);

        // previous_speed = speed_filter_info->data;
        // pub_.publish(filter_acc);

        before_time_ = now;
    }
    // 一阶滤波
    float LopPassFilter_RC_1st(float oldData, float newData, float a)
    {
        std::cout << "LopPassFilter_RC_1st a=:" << a << std::endl;
        return oldData * (1 - a) + newData * a;
    }

    // 计算比例系数a:
    float LopPassFilter_RC_1st_Factor_Cal(float deltaT, float Fcut)
    {
        return deltaT / (deltaT + 1 / (2 * PI * Fcut));
    }

private:
    ros::NodeHandle n_;
    ros::Publisher filter_pub;
    ros::Publisher state1_pub;
    ros::Subscriber sub_ak80;
    float sum_speed;
    float sum_acc_speed;
    float sum_acc_acc_speed;
    float acc_speed;
    float acc_acc_speed;
    // 先前的
    float previous_speed;
    float previous_acc_speed;
    float previous_acc_acc_speed;
    float current_speed;
    // 滤波过后的速度
    float previous_filter_vel;

    // 采样点数
    const int sample_num = 40;
    int count;
    const float PI = 3.1415926;

    // 是一个全局变量
    double before_time_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Speed_filter");

    Speed_filter Speed_filter;

    ros::Rate loop_rate(500);
    ros::spin();
    loop_rate.sleep();

    return 0;
}
