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
    bool pub_flag = false;
    multi_msgs::target_info info;

    std_msgs::Int32 jump_flag;
    ros::Publisher pub;
    ros::Publisher pub1;
    ros::Publisher pub2;
    std_msgs::Int8 simulation_jump_flag;

    // 跳跃时以0.3速度前进,目标的放大倍数，0.05为步进,上坡设置为2，跳跃设置为6
    //   double v_linear_rate=4;
    double v_linear_rate = 5;

    double vx_scale = 0, yaw_scale = 0, x_scale = 0;

    double linear_ratio, angular_ratio;
    double tar_speed, tar_yaw;

private:
    /* data */
    void callback(const sensor_msgs::Joy::ConstPtr &Joy);
    void linear_speed_updown(const sensor_msgs::Joy::ConstPtr &Joy);
    void angular_speed_updown(const sensor_msgs::Joy::ConstPtr &Joy);
    void x_tar_updown(const sensor_msgs::Joy::ConstPtr &Joy);
    //! init_zero
    void line_and_angular_zero(const sensor_msgs::Joy::ConstPtr &Joy);

    //*限止大小
    void max_min_charge(double &v);

    ros::NodeHandle n; // 实例化节点
    ros::Subscriber sub;

    double vlinear, vangular;                                                                                // todo控制速度，是通过这两个变量调整
    int axis_ang, axis_lin;                                                                                  // axes[]的键
    int en_button0, en_button1, mode_button, linear_up, linear_down, angular_up, angular_down, x_up, x_down; // button[]

    // 跳跃标志
    int jump_flag_button_select;
};

Teleop::Teleop()
{
    // 下面按键的设置，一定要根据自己的实际情况来更改
    n.param<int>("axis_linear", axis_lin, 7);       // 默认axes[1]接收速度上下
    n.param<int>("axis_angular", axis_ang, 6);      // 默认axes[2]接收航向角左右
    n.param<double>("vel_linear", vlinear, 0.0f);   // 默认线速度1.5 m/s
    n.param<double>("vel_angular", vangular, 0.0f); // 默认角速度1 单位rad/s

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

    // 发送跳跃标志
    pub1 = n.advertise<std_msgs::Int32>("/start_jump", 1);
    //simulation_jump_flag发送仿真跳跃标志
    pub2 = n.advertise<std_msgs::Int8>("/simulation_jump_flag", 1);

}

void Teleop::callback(const sensor_msgs::Joy::ConstPtr &Joy)
{
    if (Joy->buttons[en_button0] != 0 || Joy->buttons[en_button1] != 0)
    {
        ROS_INFO("buttons en0:%d, buttons en1:%d", Joy->buttons[en_button0], Joy->buttons[en_button1]);
        if ((Joy->axes[axis_lin] != 0) || (Joy->axes[axis_ang] != 0))
        {
            pub_flag = true;

            linear_speed_updown(Joy);
            angular_speed_updown(Joy);
            x_tar_updown(Joy);
            line_and_angular_zero(Joy);

            // tar_speed = Joy->axes[axis_lin]*v_linear_rate; //将游戏手柄的数据乘以你想要的速度
            tar_speed = 0;

            // tar_yaw = Joy->axes[axis_ang]*v_linear_rate; //将游戏手柄的数据乘以你想要的速度
            tar_yaw = 0;

            info.tar_x = x_scale;
            // info.tar_speed =tar_speed+vx_scale;
            info.tar_speed = tar_speed + vx_scale * v_linear_rate;
            info.tar_yaw = tar_yaw + yaw_scale;

            max_min_charge(info.tar_speed);
            max_min_charge(info.tar_yaw);
            max_min_charge(info.tar_x);
        }
        else
        {

            jump_flag.data = 0;
            simulation_jump_flag.data = 0;
            // todo
        }
    }
    else
    {
        // todo
        if (Joy->buttons[jump_flag_button_select] != 0)
        {
            jump_flag.data = 1;
            simulation_jump_flag.data = 1;
        }

        if (Joy->axes[5] < 0)
        {

            info.tar_speed = Joy->axes[1] * 0.5;
        }
    }
    ROS_INFO("v_linear_rate:%.3lf ", v_linear_rate);
}

void Teleop::linear_speed_updown(const sensor_msgs::Joy::ConstPtr &Joy)
{

    ROS_INFO("Joy->axes[axis_lin]:%.3lf ", Joy->axes[axis_lin]);

    if (Joy->buttons[linear_up])
    {

        vx_scale += linear_ratio;
        ROS_INFO("vx_scale:%.3lf ", vx_scale);
    }
    else if (Joy->buttons[linear_down])
    {

        vx_scale -= linear_ratio;
        ROS_INFO("vx_scale:%.3lf ", vx_scale);
    }
}

void Teleop::angular_speed_updown(const sensor_msgs::Joy::ConstPtr &Joy)
{

    ROS_INFO("Joy->axes[axis_ang]:%.3lf ", Joy->axes[axis_ang]);
    if (Joy->buttons[angular_up])
    {

        yaw_scale += linear_ratio;
        ROS_INFO("yaw_scale:%.3lf ", yaw_scale);
    }
    else if (Joy->buttons[angular_down])
    {

        yaw_scale -= linear_ratio;
        ROS_INFO("-yaw_scale:%.3lf ", yaw_scale);
    }
}

void Teleop::x_tar_updown(const sensor_msgs::Joy::ConstPtr &Joy)
{

    if (Joy->buttons[x_up])
    {

        x_scale += linear_ratio;
        ROS_INFO("x_scale:%.3lf ", x_scale);
    }
    else if (Joy->buttons[x_down])
    {

        x_scale -= linear_ratio;
        ROS_INFO("-x_scale:%.3lf ", x_scale);
    }
}

void Teleop::line_and_angular_zero(const sensor_msgs::Joy::ConstPtr &Joy)
{

    if (Joy->buttons[mode_button])
    {
        yaw_scale = 0;
        vx_scale = 0;
        x_scale = 0;
        ROS_INFO("line_and_angular_zero");
    }
}

void Teleop::max_min_charge(double &v)
{
    if (v > 0)
        v > 2 ? v = 2 : v;
    else if (v < 0)
        v < -2 ? v = -2 : v;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "logteleop");
    Teleop telelog;

    ros::Rate loop_rate(500);

    while (ros::ok())
    {
        // if(telelog.pub_flag == true)
        // {
        //     telelog.pub.publish(telelog.info);
        // }
        telelog.pub.publish(telelog.info);
        telelog.pub1.publish(telelog.jump_flag);
        telelog.pub2.publish(telelog.simulation_jump_flag);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
