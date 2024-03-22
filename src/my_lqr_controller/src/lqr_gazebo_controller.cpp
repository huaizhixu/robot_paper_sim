// 控制实体机器人的LQR实现
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <multi_msgs/multi_info.h>
#include <multi_msgs/target_info.h>
#include <signal.h>
#include <std_msgs/Int8.h>

// 发布
geometry_msgs::Vector3 roll_pitch_yaw;
std_msgs::Float64 wheel1_msg;
std_msgs::Float64 wheel2_msg;
Eigen::VectorXd v(3);

double roll, pitch, yaw;

//时间
double before_time_s;

float leg_position;

//lqr控制状态
multi_msgs::multi_info info_lqr;

// 目标状态
multi_msgs::target_info target_msg;
float speed_to_x = 0.0;

//停止flag
int stop_flag=0;


// lqr 参数初始化
//定义LQR控制下的参数
    Eigen::VectorXf states_lqr(4);
    Eigen::VectorXf states_rot_lqr(2);
    Eigen::VectorXf U_lqr(2);
    Eigen::VectorXf U_rot_lqr(2);
    Eigen::MatrixXf K_lqr(2,4);
    Eigen::MatrixXf K_rot_lqr(2,2);


//使用ctrl+c退出
void mySigintHandler(int sig);



void init();
//! 自己计算的roll,ritch,yaw
Eigen::VectorXd EulerianAngle(Eigen::VectorXd data);

void model_state_callback(const sensor_msgs::JointStateConstPtr &msg);

void model_imu_callback(const sensor_msgs::ImuConstPtr &msg);

void target_Callback(const multi_msgs::target_info::ConstPtr &msg);

void stop_ak_Callback(const std_msgs::Int8::ConstPtr &msg);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gazebo_control");
    ros::NodeHandle nh;

    // ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1000, model_state_callback);
    // 订阅机器人的状态
    ros::Subscriber joint_states_sub = nh.subscribe("/myrobot/joint_states", 1000, model_state_callback);
    // 订阅IMU的状态target_msg
    ros::Subscriber imu_sub = nh.subscribe("sim_imu", 1000, model_imu_callback);

    //订阅目标值
      // 订阅目标值,期望位置
    ros::Subscriber sub_target = nh.subscribe("/multi_target", 1000,target_Callback);
    ros::Subscriber sub_stop = nh.subscribe("/simulation_stop_ak_flag", 1000,stop_ak_Callback);

    // ros发布消息
    ros::Publisher joint_wheel1_pub = nh.advertise<std_msgs::Float64>("/myrobot/left_wheel_controller/command", 1000);
    ros::Publisher joint_wheel2_pub = nh.advertise<std_msgs::Float64>("/myrobot/right_wheel_controller/command", 1000);
    ros::Publisher info_pub = nh.advertise<multi_msgs::multi_info>("/myrobot_satate", 1000);
    // 发布计算的roll，pitch，yaw
    ros::Publisher roll_pub = nh.advertise<geometry_msgs::Vector3>("/sim_roll_pitch_yaw", 1000);

    // ctrl+c
    signal(SIGINT, mySigintHandler);

    // 初始化
    leg_position=0;
    init();
    wheel1_msg.data = 0.0;
    wheel2_msg.data = 0.0;

    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        // ROS_INFO_STREAM("hello,world");
        double now = ros::Time::now().toSec();
        double dt=now - before_time_s;
        std::cout << "time:" << (now - before_time_s) << std::endl;
        speed_to_x+= target_msg.tar_speed * dt;
        //计算平衡控制量
        states_lqr<< info_lqr.x-target_msg.tar_x-speed_to_x, info_lqr.dx-target_msg.tar_speed, info_lqr.theta-target_msg.tar_pitch, info_lqr.dtheta;
        U_lqr = K_lqr * states_lqr;
        ROS_INFO_STREAM("K_lqr: "<<K_lqr);
        ROS_INFO_STREAM("U_lqr: "<<U_lqr);
        //转向控制量
        states_rot_lqr<<info_lqr.delta_yaw-target_msg.tar_yaw,info_lqr.ddelta_yaw;
        U_rot_lqr=K_rot_lqr*states_rot_lqr;
        ROS_INFO_STREAM("U_rot_lqr: "<<U_rot_lqr);

        // 发布控制量
        wheel1_msg.data = U_lqr(0)-U_rot_lqr(0);
        wheel2_msg.data = U_lqr(1)-U_rot_lqr(1);
        // wheel1_msg.data = U_lqr(0);
        // wheel2_msg.data = U_lqr(1);
        // wheel1_msg.data = 0;
        // wheel2_msg.data = 0;

        // 发布消息
        //站立过程平衡
        if(leg_position<0.8){
        wheel1_msg.data = 0;
        wheel2_msg.data = 0;
        }
        //停止ak电机
        if(stop_flag==1){
        wheel1_msg.data = 0;
        wheel2_msg.data = 0;
        }
        joint_wheel1_pub.publish(wheel1_msg);
        joint_wheel2_pub.publish(wheel2_msg);
        info_pub.publish(info_lqr);
        // 发布roll，pitch，yaw
        roll_pub.publish(roll_pitch_yaw);
        ros::spinOnce();
        loop_rate.sleep();
        before_time_s = now;
    }
    return 0;
}

void model_state_callback(const sensor_msgs::JointStateConstPtr &msg)
{
    float pos[8], vel[8], eff[8];
    for (int i = 0; i < 8; i++)
    {
        pos[i] = msg->position[i];
        vel[i] = msg->velocity[i];
        eff[i] = msg->effort[i];
    }
    info_lqr.x=-((pos[4]+pos[5])/2)*0.06;
    info_lqr.dx=-((vel[4]+vel[5])/2)*0.06;
    leg_position=pos[0];

    ROS_INFO_STREAM("model_state_callback");

    // ROS_INFO("pos: %f %f %f %f %f %f", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
    // ROS_INFO("vel: %f %f %f %f %f %f", vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]);
    // ROS_INFO("eff: %f %f %f %f %f %f", eff[0], eff[1], eff[2], eff[3], eff[4], eff[5]);
}

void model_imu_callback(const sensor_msgs::ImuConstPtr &msg)
{
    // float x,y,z,w;
    Eigen::VectorXd data(4);
    data << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
    v = EulerianAngle(data);

    // 定义一个四元数quadf
    // tf::Quaternion quat;
    // // 将四元数赋值给quadf
    // tf::quaternionMsgToTF(msg->orientation, quat);
    // // 将四元数转换为欧拉角
    // double roll, pitch, yaw;
    // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // ROS_INFO("roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
    ROS_INFO_STREAM("model_imu_callback");


        tf::Quaternion my_quaternion(msg->orientation.x,
                        msg->orientation.y,
                        msg->orientation.z,
                        msg->orientation.w);
        tf::Matrix3x3 m(my_quaternion);
        m.getRPY(roll, pitch, yaw);


    // roll_pitch_yaw.x = roll;
    // roll_pitch_yaw.y = pitch;
    // roll_pitch_yaw.z = yaw;
    // 发布
    roll_pitch_yaw.x = -v(0);
    roll_pitch_yaw.y = v(1);
    roll_pitch_yaw.z = v(2);

    // 初始化
    info_lqr.theta=-roll;
    info_lqr.dtheta = -msg->angular_velocity.x;
    info_lqr.delta_yaw = v(2);
    info_lqr.ddelta_yaw=msg->angular_velocity.z;
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


void init(){
        // 添加x的控制量
        // K_lqr << -1.6093, -2.5332, -11.6930, -1.9765,
        //          -1.6093, -2.5332, -11.6930, -1.9765;
        K_lqr << 0, -2.5332, -11.6930, -1.9765,
                 0, -2.5332, -11.6930, -1.9765;

        K_rot_lqr << 2.0817, 0.4228,
                    -2.0817, -0.4228;
        U_lqr << 0, 0;
        U_rot_lqr << 0, 0;
        states_lqr << 0, 0, 0, 0;
        states_rot_lqr << 0, 0;
        target_msg.tar_x=0;
        target_msg.tar_speed=0;
        target_msg.tar_pitch=0;
        target_msg.tar_yaw=0;

}


void target_Callback(const multi_msgs::target_info::ConstPtr &msg){
    target_msg.tar_x=msg->tar_x;
    target_msg.tar_speed=msg->tar_speed;
    // target_msg.tar_pitch=msg->tar_pitch;
    target_msg.tar_pitch=msg->tar_x;
    target_msg.tar_yaw=msg->tar_yaw;
}


void stop_ak_Callback(const std_msgs::Int8::ConstPtr &msg){
    stop_flag=msg->data;
}


void mySigintHandler(int sig)
{

    ros::shutdown();
}