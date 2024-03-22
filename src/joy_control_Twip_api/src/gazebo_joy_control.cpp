#include <iostream>
#include <multi_msgs/target_info.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
using namespace std;

class Teleop
{
public:
    Teleop();

public:
    multi_msgs::target_info info;
        std_msgs::Int8 simulation_jump_flag;

    ros::Publisher pub;
    ros::Publisher pub1;

    // 跳跃时以0.3速度前进,目标的放大倍数，0.05为步进,上坡设置为2，跳跃设置为6
    //double v_linear_rate=4;
    double v_linear_rate = 2;

    double vx_scale = 0, yaw_scale = 0, x_scale = 0;

    double linear_ratio, angular_ratio;
    double tar_speed, tar_yaw;

private:
    /* data */
    void callback(const sensor_msgs::Joy::ConstPtr &Joy);
    void linear_speed_updown(const sensor_msgs::Joy::ConstPtr &Joy);
    void angular_speed_updown(const sensor_msgs::Joy::ConstPtr &Joy);

    //*限止大小
    void max_min_charge(double &v);

    ros::NodeHandle n; // 实例化节点
    ros::Subscriber sub;

    double vlinear, vangular;                                                                                // todo控制速度，是通过这两个变量调整
    int axis_ang, axis_lin;
    //🕹摇杆控制球体运动
    int axis_0, axis_1;

    int en_button0, en_button1, mode_button, linear_up, linear_down, angular_up, angular_down, x_up, x_down; // button[]
        // 跳跃标志
    int jump_flag_button_select;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_joy_control");
    Teleop telelog;

    ros::Rate loop_rate(500);

    while (ros::ok())
    {
        telelog.pub.publish(telelog.info);
        telelog.pub1.publish(telelog.simulation_jump_flag);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}



Teleop::Teleop()
{
    // 下面按键的设置，一定要根据自己的实际情况来更改
    n.param<int>("axis_linear", axis_lin, 7);       // 默认axes[1]接收速度上下
    n.param<int>("axis_angular", axis_ang, 6);      // 默认axes[2]接收航向角左右
    n.param<double>("vel_linear", vlinear, 0.0f);   // 默认线速度1.5 m/s
    n.param<double>("vel_angular", vangular, 0.0f); // 默认角速度1 单位rad/s

    //🕹摇杆控制球体运动
    n.param<int>("axis_0", axis_0, 0);
    n.param<int>("axis_1", axis_1, 1);

    n.param<int>("linear_up", linear_up, 2);       // 速度增加
    n.param<int>("linear_down", linear_down, 3);   // 速度减少
    n.param<int>("angular_up", angular_up, 0);     // 航向角增加
    n.param<int>("angular_down", angular_down, 1); // 航向角减少

    n.param<double>("linear_ratio", linear_ratio, 0.05);

    n.param<int>("en_button", en_button0, 5); //!  enable
    n.param<int>("en_button", en_button1, 4); // todo enable

    n.param<int>("x_up", x_up, 9);
    n.param<int>("x_down", x_down, 10);

    n.param<int>("en_button", mode_button, 8); // *init_zero target speed
    // 按键button select 6
    n.param<int>("jump_flag_button_select", jump_flag_button_select, 6);


    pub = n.advertise<multi_msgs::target_info>("/multi_target", 1000);         // 将速度发送给lqr
    sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, &Teleop::callback, this); // 订阅游戏手柄发来的数据
        //simulation_jump_flag发送仿真跳跃标志
    pub1 = n.advertise<std_msgs::Int8>("/simulation_jump_flag", 1);
    ROS_INFO("Teleop");
}

void Teleop::callback(const sensor_msgs::Joy::ConstPtr &Joy)
{
    if (Joy->buttons[en_button0] != 0)
    {
        ROS_INFO("en_button0");
        if ((Joy->axes[axis_0] != 0) || (Joy->axes[axis_1] != 0))
        {

            linear_speed_updown(Joy);
            angular_speed_updown(Joy);
            info.tar_speed = vx_scale ;
            info.tar_yaw += yaw_scale;

            max_min_charge(info.tar_speed);
            max_min_charge(info.tar_yaw);
            max_min_charge(info.tar_x);
        }
        else
        {
            info.tar_speed = 0;
            // info.tar_yaw = 0;
            info.tar_x = 0;
        if(Joy->buttons[jump_flag_button_select] != 0)
        {
            simulation_jump_flag.data = 1;
        }else{
            simulation_jump_flag.data = 0;
        }
        }


    }
    else
    {
        info.tar_speed = 0;
        // info.tar_yaw = 0;
        info.tar_x = 0;
    }
}

void Teleop::linear_speed_updown(const sensor_msgs::Joy::ConstPtr &Joy)
{

        vx_scale =Joy->axes[axis_1]*1;

}

void Teleop::angular_speed_updown(const sensor_msgs::Joy::ConstPtr &Joy)
{
        yaw_scale = Joy->axes[axis_0]*0.05;

}


void Teleop::max_min_charge(double &v)
{
    if (v > 0)
        v > 3 ? v = 3 : v;
    else if (v < 0)
        v < -3 ? v = -3 : v;
}
