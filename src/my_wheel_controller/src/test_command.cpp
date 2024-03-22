#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

/**
 * This tutorial  over the ROS system.
 */
int main(int argc, char **argv)
{
    // 初始化ROS，指定节点名称为“talker”，节点 名称要保持唯一性。名称定义参考
    ros::init(argc, argv, "wheel");

    // 节点实例化
    ros::NodeHandle n;

    ros::Publisher chatter_pub[3];

    // 发布腿部的命令,发布一个消息类型为......，命名话题。
    // 定义消息队列大小为1000，即超过1000条消息之后，旧的消息就会丢弃。
    chatter_pub[0] = n.advertise<std_msgs::Float64MultiArray>("/myrobot/leg1423_Effort_controller/command", 1000);
    chatter_pub[1] = n.advertise<std_msgs::Float64MultiArray>("/myrobot/wheel34_Effort_controller/command", 1000);
    chatter_pub[2] = n.advertise<std_msgs::Float64MultiArray>("/myrobot/wheel12_Effort_controller/command", 1000);

    // 指定发布消息的频率，这里指10Hz，也即每秒10次
    ros::Rate loop_rate(10);
    // 通过 Rate::sleep()来处理睡眠的时间来控制对应的发布频率。

    int flag_leg_mode = 0;

    while (ros::ok())
    {

        std_msgs::Float64MultiArray msg_leg;
        std_msgs::Float64MultiArray msg_ball_wheel;
        // 初始化腿部的关节位置
        if (flag_leg_mode == 0)
        {
            for (int i = 0; i <= 3; ++i)
            {
                msg_leg.data.push_back(0);
            }

            // 打印输出
            for (int i = 0; i <= 3; ++i)
            {
                ROS_INFO("/myrobot/leg1423_Effort_controller/command[%d]:%f", i, msg_leg.data.at(i));
            }
        }

        // 测试机器人站立
        if (flag_leg_mode == 1)
        {
            msg_leg.data.clear();
            msg_leg.data.push_back(1.5);
            msg_leg.data.push_back(-1);
            msg_leg.data.push_back(-1);
            msg_leg.data.push_back(1.5);
            for (int i = 0; i <= 3; ++i)
            {
                ROS_INFO("/myrobot/leg1423_Effort_controller/command[%d]:%f", i, msg_leg.data.at(i));
            }
        }

        // 测试机器人站立
        if (flag_leg_mode == 2)
        {
            msg_leg.data.clear();
            for (int i = 0; i <= 3; ++i)
            {
                msg_leg.data.push_back(0);
            }
            for (int i = 0; i <= 3; ++i)
            {
                ROS_INFO("/myrobot/leg1423_Effort_controller/command[%d]:%f", i, msg_leg.data.at(i));
            }
        }

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        chatter_pub[0].publish(msg_leg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}