// 控制实体机器人的VMC实现
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <multi_msgs/multi_info.h>
#include <multi_msgs/target_info.h>
#include <signal.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>
#include "my_lqr_controller/leg_kinematics.h"
#include <cmath>

// 机器人🤖的参数
//  连杆长度L1，L2，L3
const float L1 = 0.05;
// 短腿的长度
const float L2 = 0.09;
const float L3 = 0.18;

const float M = 10.00;
const float g = 9.8;

// 位置
float pos[8], vel[8], eff[8];

// VMC算法的参数
const float Kpy1 = 1300.0; // 弹性系数
const float Kdy1 = 30;   // 阻尼系数

const float Kpx1 = -10.0; // 弹性系数
const float Kdx1 = 0.0;   // 阻尼系数

// 设置目标高度，速度
float height_set = 0.13;
float heightx_set = 0.0;
float speed_set = 0.0;
// 当前的高度，速度
float current_height = 0.0;
float current_speed = 0.0;

// 横滚角度方向的设置
const float Kp2 = 0.0; // 弹性系数
const float Kd2 = 0.0; // 阻尼系数
float roll_set = 0.0;
float roll_speed_set = 0.0;
// 当前的横滚角度，速度
float current_roll = 0.0;
float current_roll_speed = 0.0;

// 发布的力矩
float calcu_tau = 0.0;

// imu的数据
Eigen::VectorXd v(3);
geometry_msgs::Vector3 roll_pitch_yaw;
double roll, pitch, yaw;

// debug_view
geometry_msgs::Vector3 debug_view;

std_msgs::Float64MultiArray msg_leg;

// 函数
// 使用ctrl+c退出
void mySigintHandler(int sig);

//! 自己计算的roll,ritch,yaw
Eigen::VectorXd EulerianAngle(Eigen::VectorXd data);

void vmc_model_state_callback(const sensor_msgs::JointStateConstPtr &msg);

void vmc_imu_callback(const sensor_msgs::ImuConstPtr &msg);

// 计算雅可比矩阵
float calculate_jacobi(double x);
double hight_robot(double q);

// 通过弹簧计算力矩的大小
float calculate_tau(const float Kp1, const float Kd1, const float M,
                    float hight_current, float hight_set, float v, float Jacobi);

float calculatex_tau(const float Kp1, const float Kd1, const float M,
                    float hight_current, float hight_set, float v, float Jacobi);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vmc_control");
    ros::NodeHandle nh;

    // 订阅机器人的状态
    ros::Subscriber joint_states_sub = nh.subscribe("/myrobot/joint_states", 1000, vmc_model_state_callback);
    // 订阅IMU的状态target_msg
    ros::Subscriber imu_sub = nh.subscribe("sim_imu", 1000, vmc_imu_callback);

    // 发布
    ros::Publisher leg_pub = nh.advertise<std_msgs::Float64MultiArray>("/myrobot/leg1423_Effort_controller/command", 1000);

    // ros::Publisher leg1_joint_pub = nh.advertise<std_msgs::Float64>("/myrobot/leg1_joint_controller/command", 1000);
    // ros::Publisher leg2_joint_pub = nh.advertise<std_msgs::Float64>("/myrobot/leg2_joint_controller/command", 1000);
    // ros::Publisher leg3_joint_pub = nh.advertise<std_msgs::Float64>("/myrobot/leg3_joint_controller/command", 1000);
    // ros::Publisher leg4_joint_pub = nh.advertise<std_msgs::Float64>("/myrobot/leg4_joint_controller/command", 1000);
    // 仿真计算力矩
    ros::Publisher vmc_tau_pub = nh.advertise<std_msgs::Float64>("/simulation_vmc_tau", 1000);
    ros::Publisher vmc_imu_pub = nh.advertise<geometry_msgs::Vector3>("/simulation_vmc_imu", 1000);

    ros::Publisher vmc_debug_pub = nh.advertise<geometry_msgs::Vector3>("/vmc_debug_view", 1000);
    // ctrl+c
    signal(SIGINT, mySigintHandler);

    //全部腿的运动学对象获取
    LegKin leg_calcu = LegKin();

    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        // std_msgs::Float64MultiArray msg_leg;
        std_msgs::Float64 vmc_tau;
        std_msgs::Float64 leg_joint[4];

        // 计算力矩
        // TODO：计算力矩 计算当前的高度
        
        // ROS_INFO("height_set....  :%lf",height_set);
        // float Jacobi = calculate_jacobi(pos[0] - 0.64);
        // ROS_INFO("Jacobi....  :%lf", Jacobi);
        // current_speed = 0.12 * vel[0];
        // debug_view.x = Jacobi;
        // Jacobi = 0.12;

        msg_leg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg_leg.layout.dim[0].label = "leg1423_Effort";
        msg_leg.layout.dim[0].size = 4;
        msg_leg.layout.dim[0].stride = 1;


        //全部腿部的运动学测试一下
        Eigen::Vector2d q =leg_calcu.ForKin(pos[0] - 0.64,-pos[1] - 0.64);

        ROS_INFO("x:q(0):%f,y:q(1):%f",q(0),q(1));

        current_height = -q(1);

        ROS_INFO("current_hight....  :%lf", current_height);
        ROS_INFO("height_set....  :%lf",height_set);

        float Jacobi = calculate_jacobi(pos[0] - 0.64);

        ROS_INFO("Jacobi....  :%lf", Jacobi);

        current_speed = 0.12 * vel[0];
        debug_view.x = Jacobi;
        Jacobi = 0.12;


        // 计算当前力矩
        float calcu_tau1 = calculate_tau(Kpy1, Kdy1, M, current_height, height_set, current_speed, Jacobi);
        float calcu_tau3 = calculatex_tau(Kpx1, Kdx1, M, q(0), heightx_set, current_speed, Jacobi);

        float calcu_tau2 = calculate_tau(Kp2, Kd2, M, current_roll, roll_set, current_roll_speed, Jacobi);
        // calcu_tau=calcu_tau1+calcu_tau2;
        debug_view.y = calcu_tau1;
        // calculate_tau(Kpy1, Kdy1, M, current_height, height_set, current_speed, Jacobi)
        ROS_INFO("calculate_tau1......  :%f", calculate_tau(Kpy1, Kdy1, M, current_height, height_set, current_speed, Jacobi));
        ROS_INFO("calculate_tau1  :%f", calcu_tau1);
        calcu_tau = calcu_tau1;


        ROS_INFO("publish_calculate_tau  :%f", calcu_tau);

        vmc_tau.data = calcu_tau;

        msg_leg.data.clear();
        msg_leg.data.push_back(calcu_tau+calcu_tau3);
        msg_leg.data.push_back(-calcu_tau-calcu_tau3);
        msg_leg.data.push_back(-calcu_tau-calcu_tau3);
        msg_leg.data.push_back(calcu_tau+calcu_tau3);

        // leg_joint[0].data = calcu_tau;
        // leg_joint[1].data = -calcu_tau;
        // leg_joint[2].data = calcu_tau;
        // leg_joint[3].data = -calcu_tau;

        // 发布
        leg_pub.publish(msg_leg);
        vmc_tau_pub.publish(vmc_tau);
        vmc_imu_pub.publish(roll_pitch_yaw);
        vmc_debug_pub.publish(debug_view);
        // leg1_joint_pub.publish(leg_joint[0]);
        // leg2_joint_pub.publish(leg_joint[1]);
        // leg3_joint_pub.publish(leg_joint[2]);
        // leg4_joint_pub.publish(leg_joint[3]);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void mySigintHandler(int sig)
{
    ros::shutdown();
}

//! 自己计算的roll,ritch,yaw
Eigen::VectorXd EulerianAngle(Eigen::VectorXd data)
{

    Eigen::VectorXd ans(3);

    double q2sqr = data(2) * data(2);
    double t0 = -2.0 * (q2sqr + data(3) * data(3)) + 1.0;
    double t1 = +2.0 * (data(1) * data(2) + data(0) * data(3));
    double t2 = -2.0 * (data(1) * data(3) - data(0) * data(2));
    double t3 = +2.0 * (data(2) * data(3) + data(0) * data(1));
    double t4 = -2.0 * (data(1) * data(1) + q2sqr) + 1.0;

    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    // roll  pitch yaw
    ans(0) = atan2(t3, t4);
    ans(1) = asin(t2);
    ans(2) = atan2(t1, t0);

    return ans;
}

void vmc_model_state_callback(const sensor_msgs::JointStateConstPtr &msg)
{

    for (int i = 0; i < 8; i++)
    {
        pos[i] = msg->position[i];
        vel[i] = msg->velocity[i];
        eff[i] = msg->effort[i];
    }

    ROS_INFO_STREAM_THROTTLE(7, "\033[45;37m vmc_model_state_callback \033[0m");
}

void vmc_imu_callback(const sensor_msgs::ImuConstPtr &msg)
{
    // float x,y,z,w;
    Eigen::VectorXd data(4);
    data << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
    v = EulerianAngle(data);

    ROS_INFO_STREAM_THROTTLE(7, "\033[45;37m vmc_imu_callback \033[0m");

    tf::Quaternion my_quaternion(msg->orientation.x,
                                 msg->orientation.y,
                                 msg->orientation.z,
                                 msg->orientation.w);
    tf::Matrix3x3 m(my_quaternion);
    m.getRPY(roll, pitch, yaw);

    // 发布
    roll_pitch_yaw.x = -v(0);
    roll_pitch_yaw.y = v(1);
    // roll_pitch_yaw.z = v(2);
    roll_pitch_yaw.z = msg->angular_velocity.y;
    // 计算力矩使用的
    current_roll = v(1);
    current_roll_speed = msg->angular_velocity.y;
    // 初始化
    // info_lqr.theta=-roll;
    // info_lqr.dtheta = -msg->angular_velocity.x;
    // info_lqr.delta_yaw = v(2);
    // info_lqr.ddelta_yaw=msg->angular_velocity.z;
}

// 根据角度计算当前的高度
double hight_robot(double q)
{
    // ROS_INFO("current_hight  :%lf", sqrt(L3 * L3 - pow((L1 + L2 * cos(q)), 2)) + L2 * sin(q));
    return sqrt(L3 * L3 - pow((L1 + L2 * cos(q)), 2)) + L2 * sin(q);
}

float calculate_tau(const float Kp1, const float Kd1, const float M,
                    float hight_current, float hight_set, float v, float Jacobi)
{
    float tau;
    float P_tau; // 弹簧虚拟力v
    P_tau = (Kp1 * (hight_set - hight_current) - Kd1 * v + M * g) / 2;
    tau = Jacobi * P_tau;
    ROS_INFO("ycalculate_highty_P_tau:%f", P_tau);
    ROS_INFO("ycalculate_highty_tau:%f", tau);

    return tau;
}

float calculatex_tau(const float Kp1, const float Kd1, const float M,
                    float hight_current, float hight_set, float v, float Jacobi)
{

    float tau;
    float P_tau; // 弹簧虚拟力
    P_tau = (Kp1 * (hight_set - hight_current) - Kd1 * v)/ 2;
    tau = Jacobi * P_tau;
    ROS_INFO("xcalculate_hightx_P_tau:%f", P_tau);
    ROS_INFO("xcalculate_hightx_tau:%f", tau);
    return tau;
}





float calculate_jacobi(double x)
{
    float Jacobi;
    //  (9*cos(x))/100 + (9*sin(x)*((9*cos(x))/100 + 1/20))/(100*(81/2500 - ((9*cos(x))/100 + 1/20)^2)^(1/2))
    Jacobi = (9 * cos(x)) / 100 + (9 * sin(x) * ((9 * cos(x)) / 100 + 1 / 20)) / (100 * pow((pow(81 / 2500 - ((9 * cos(x)) / 100 + 1 / 20), 2)), 0.5));
    // ROS_INFO("calculate_jacobi_funion  :%f", Jacobi);
    return Jacobi;
}
