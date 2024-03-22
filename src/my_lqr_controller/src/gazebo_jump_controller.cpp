#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <signal.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>

float leg1423_Position = 0;

std_msgs::Float64MultiArray msg_leg;
std_msgs::Int8 sim_jump_flag;
int leg_stop;
int has_hight;

float pos[8], vel[8], eff[8];

void mySigintHandler(int sig);
void simulation_start_jumpCallback(const std_msgs::Int8::ConstPtr &msg);
void jump_model_state_callback(const sensor_msgs::JointStateConstPtr &msg);

int main(int argc, char **argv)
{
    // 初始化ROS，指定节点名称为“talker”，节点 名称要保持唯一性。名称定义参考
    ros::init(argc, argv, "leg1423_jump");

    // 节点实例化
    ros::NodeHandle n;

    ros::Publisher chatter_pub[4];

    // ctrl+c
    signal(SIGINT, mySigintHandler);

    // 发布腿部的命令,发布一个消息类型为......，命名话题。
    // 定义消息队列大小为1000，即超过1000条消息之后，旧的消息就会丢弃。
    chatter_pub[0] = n.advertise<std_msgs::Float64MultiArray>("/myrobot/leg1423_Effort_controller/command", 1000);

    // 订阅
    ros::Subscriber simulation_jump_flage = n.subscribe("simulation_jump_flag", 1000, simulation_start_jumpCallback);
    ros::Subscriber simulation_joint_states = n.subscribe("/myrobot/joint_states", 1000, jump_model_state_callback);

    // 指定发布消息的频率，这里指100Hz，也即每秒100次
    ros::Rate loop_rate(500);

    int flag_leg_mode = 0;
    sim_jump_flag.data = 0;
    leg_stop = 0;
    has_hight = 0;
    msg_leg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_leg.layout.dim[0].label = "leg1423_Effort";
    msg_leg.layout.dim[0].size = 4;
    msg_leg.layout.dim[0].stride = 1;

    while (ros::ok())
    {

        if (pos[0] >= 1.98)
        {
            has_hight = 1;
        }
        // sim_jump_flag.data接收到开始跳跃的信号
        if (sim_jump_flag.data == 1 && has_hight == 0)
        {

            msg_leg.data.clear();
            msg_leg.data.push_back(2);
            msg_leg.data.push_back(-2);
            msg_leg.data.push_back(-2);
            msg_leg.data.push_back(2);
            ROS_INFO_STREAM("has_hight........"<< has_hight);
            ROS_INFO_STREAM("pos[0]........:" << pos[0]);
        }

        if (sim_jump_flag.data == 0 && has_hight == 1)
        {
            msg_leg.data.clear();
            msg_leg.data.push_back(1);
            msg_leg.data.push_back(-1);
            msg_leg.data.push_back(-1);
            msg_leg.data.push_back(1);
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

void simulation_start_jumpCallback(const std_msgs::Int8::ConstPtr &msg)
{
    sim_jump_flag.data = msg->data;
}

void jump_model_state_callback(const sensor_msgs::JointStateConstPtr &msg)
{
    for (int i = 0; i < 8; i++)
    {
        pos[i] = msg->position[i];
        vel[i] = msg->velocity[i];
        eff[i] = msg->effort[i];
    }
    
}