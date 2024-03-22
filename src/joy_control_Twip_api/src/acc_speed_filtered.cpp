#include <geometry_msgs/Vector3.h>
#include <multi_msgs/ak80_info.h>
#include <multi_msgs/multi_info.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

class Acc_speed_filter
{
public:
    Acc_speed_filter() : n_("~")
    {
        before_time_ = 0.0;
        previous_speed = 0.0;
        sum_acc_speed = 0.0;
        acc_acc_speed = 0.0;
        count = 0;

        pub_ = n_.advertise<std_msgs::Float32>("/filter_acc_pub", 1000);
        sub_ = n_.subscribe("/acc_acc_original_info", 1000, &Acc_speed_filter::ak_speed_Callback, this);
    }

    void ak_speed_Callback(const std_msgs::Float32::ConstPtr &speed_filter_info)
    {
        double now = ros::Time::now().toSec();
        std::cout << "time:" << now - before_time_ << std::endl;
        // 一阶滤波
        std_msgs::Float32 filter_acc;
        acc_acc_speed = (speed_filter_info->data - previous_speed) / 0.002;

        LopPassFilter_RC_1st_Factor_Cal(now - before_time_, 10);
        float B = LopPassFilter_RC_1st_Factor_Cal(0.002, 50);
        std::cout << "B:" << LopPassFilter_RC_1st_Factor_Cal(0.002, 50) << std::endl;

        filter_acc.data = LopPassFilter_RC_1st(previous_speed, speed_filter_info->data, B);

        previous_speed = speed_filter_info->data;
        pub_.publish(filter_acc);
        // 均值滤波
        //  sum_acc_speed += speed_filter_info->data;
        //  acc_acc_speed = (speed_filter_info->data - previous_speed) / (now - before_time_);
        //  previous_speed = speed_filter_info->data;

        // count++;
        // if (count >= sample_num)
        // {
        //     std_msgs::Float32 filter_acc;
        //     filter_acc.data = sum_acc_speed / sample_num;
        //     if(filter_acc.data>25000||filter_acc.data<-25000)
        //     {
        //         filter_acc.data=0;
        //     }

        //     pub_.publish(filter_acc);
        //     count = 0;
        //     sum_acc_speed = 0.0;
        // }
        before_time_ = now;
    }
    float first_Order_Filter(float data)
    {
        float filter_data;
        filter_data = A * data + (1 - A) * previous_speed;
        std::cout << "first_Order_Filter" << std::endl;
        previous_speed = filter_data;
        return filter_data;
    }

    float LopPassFilter_RC_1st(float oldData, float newData, float a)
    {
        std::cout << "LopPassFilter_RC_1st" << std::endl;
        return oldData * (1 - a) + newData * a;
    }

    // 计算比例系数a:
    float LopPassFilter_RC_1st_Factor_Cal(float deltaT, float Fcut)
    {
        return deltaT / (deltaT + 1 / (2 * PI * Fcut));
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

    double before_time_;
    float previous_speed;
    float sum_acc_speed;
    float acc_acc_speed;

    // 采样点数
    const int sample_num = 20;
    int count;
    const float A = 0.07;
    const float PI = 3.1415926;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "acc_filter");

    Acc_speed_filter Acc_speed_filter;

    ros::Rate loop_rate(500);
    ros::spin();
    loop_rate.sleep();

    return 0;
}