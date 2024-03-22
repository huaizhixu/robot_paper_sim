#include "ros/ros.h"
#include "std_msgs/String.h"
#include <signal.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>

float leg1423_Position = 0;

std_msgs::Float64MultiArray msg_leg;
int stand_flag = 0;

void mySigintHandler(int sig);

int main(int argc, char **argv)
{
    // 初始化ROS，指定节点名称为“talker”，节点 名称要保持唯一性。名称定义参考
    ros::init(argc, argv, "leg1423_Effort");

    // 节点实例化
    ros::NodeHandle n;

    ros::Publisher chatter_pub[4];

    // ctrl+c
    signal(SIGINT, mySigintHandler);

    // 发布腿部的命令,发布一个消息类型为......，命名话题。
    // 定义消息队列大小为1000，即超过1000条消息之后，旧的消息就会丢弃。
    chatter_pub[0] = n.advertise<std_msgs::Float64MultiArray>("/myrobot/leg1423_Effort_controller/command", 1000);
    chatter_pub[1] = n.advertise<std_msgs::Float64MultiArray>("/myrobot/wheel34_Effort_controller/command", 1000);
    chatter_pub[2] = n.advertise<std_msgs::Float64MultiArray>("/myrobot/wheel12_Effort_controller/command", 1000);
    chatter_pub[3] = n.advertise<std_msgs::Int8>("/simulation_stand_flag", 1000);
    // 指定发布消息的频率，这里指100Hz，也即每秒100次
    ros::Rate loop_rate(100);
    // 通过 Rate::sleep()来处理睡眠的时间来控制对应的发布频率。

    int flag_leg_mode = 0;
    msg_leg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_leg.layout.dim[0].label = "leg1423_Effort";
    msg_leg.layout.dim[0].size = 4;
    msg_leg.layout.dim[0].stride = 1;

    while (ros::ok())
    {
        std_msgs::Int8 msg_stand_flag;

        // 初始化腿部的关节位置

        if (flag_leg_mode < 20)
        {
            msg_leg.data.clear();
            msg_leg.data.push_back(0);
            msg_leg.data.push_back(0);
            msg_leg.data.push_back(0);
            msg_leg.data.push_back(0);
        }

        // 测试机器人站立
        if (flag_leg_mode > 30)
        {
            if (leg1423_Position > 2)
            {
                msg_leg.data.clear();
                msg_leg.data.push_back(2);
                msg_leg.data.push_back(-2);
                msg_leg.data.push_back(-2);
                msg_leg.data.push_back(2);
            }
            else
            {

                msg_leg.data.clear();
                msg_leg.data.push_back(leg1423_Position);
                msg_leg.data.push_back(-leg1423_Position);
                msg_leg.data.push_back(-leg1423_Position);
                msg_leg.data.push_back(leg1423_Position);
                leg1423_Position += 0.007;
            }
        }
        flag_leg_mode++;
        if (flag_leg_mode > 30)
        {
            flag_leg_mode = 33;
        }

        chatter_pub[0].publish(msg_leg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void mySigintHandler(int sig)
{
    ros::shutdown();
}