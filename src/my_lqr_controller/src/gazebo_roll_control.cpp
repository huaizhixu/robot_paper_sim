#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <signal.h>
#include <tf/tf.h>

class Roll_Controller
{
public:
    Roll_Controller()
    {
        publisher_ = nh_.advertise<std_msgs::Float64>("yuquan", 1000);
        subscriber_ = nh_.subscribe("sim_roll_pitch_yaw", 1000, &Roll_Controller::imu_Callback, this);
    }
    virtual ~Roll_Controller() {}

    // 发布消息。
    void publishMessage()
    {
        // 创建一个新的消息。
        std_msgs::Float64 message;

        // 设置消息的内容。
        message.data = roll;

        // 发布消息。
        publisher_.publish(message);
    }

private:
    // 订阅者。
    ros::Subscriber subscriber_;
    // 发布者。
    ros::Publisher publisher_;

    ros::NodeHandle nh_;

    double roll, pitch, yaw;

    // 回调函数。
    void imu_Callback(const geometry_msgs::Vector3::ConstPtr &msg)
    {
        roll = msg->y;
        pitch = msg->x;
        yaw = msg->z;
        ROS_INFO("roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roll_gazebo");

    Roll_Controller my_roll_controller;

    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        my_roll_controller.publishMessage();
        ros::spinOnce();
        loop_rate.sleep();
    }
    // 循环等待 ROS 消息。
    // ros::spin();

    return 0;
}
