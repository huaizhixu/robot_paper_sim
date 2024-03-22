/*
 * @Author: yuquan xu
 * @Date: 2023-02-10 10:43:52
 * @Last Modified by: yuquan xu
 * @Last Modified time: 2023-07-23 09:39:12
 */

// 多传感器，将多个信号订阅到一个节点并发布
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <multi_msgs/ak80_info.h>
#include <multi_msgs/multi_info.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>

#define PI 3.1415926

using namespace sensor_msgs;
using namespace Eigen;

ros::Publisher multi_msg_pub;
multi_msgs::multi_info info;

ros::Publisher myimuinfo_pub;

geometry_msgs::Vector3 myimuinfo;

sensor_msgs::Imu imuinfo;
multi_msgs::ak80_info ak80info;
const double wheel_radius = 0.06;
double pitch, yaw, roll, current_yaw;
float dtheta, ddelta_yaw;

float motor_13_pos, motor_14_pos;

Eigen::VectorXd v(3);

// bool flag=true;
//! 自己计算的roll,ritch,yaw
Eigen::VectorXd EulerianAngle(Eigen::VectorXd data);

void imu_Callback(const sensor_msgs::ImuConstPtr &imu_data);
void ak80_Callback(const multi_msgs::ak80_info::ConstPtr &motor_ak80_info);

//! 增加计算yaw的圈数；当yaw到180度yaw正负号发生突变
int yaw_count = 0;
float previous_yaw = 0.0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multiinfo");

  ros::NodeHandle nh;

  // 订阅IMU的信息，整合到multi_msg,仿真的话题为/myrobot/sim_imu，实际的imu为/imu
  ros::Subscriber sub_imu = nh.subscribe("/imu", 1000, imu_Callback);

  // 订阅电机的状态 话题为：motor_ak80_info
  ros::Subscriber sub_ak80 = nh.subscribe("/motor_ak80_info", 1000, ak80_Callback);

  // 发布订阅的整合信息
  multi_msg_pub = nh.advertise<multi_msgs::multi_info>("/multi_info_pub", 1000);

  myimuinfo_pub = nh.advertise<geometry_msgs::Vector3>("/myimu_info_pub", 1000);

  ros::Rate loop_rate(500); // 100HZ
  while (ros::ok())
  {

    // 此处的电机位置为镜像安装，所以求取平均的x方向相反,14电机正转为正
    //  info.x=(ak80info.pos[1]-motor_14_pos-ak80info.pos[0]+motor_13_pos)/2*wheel_radius;
    info.x = (ak80info.pos[1] + ak80info.pos[0]) / 2 * wheel_radius;

    ROS_INFO(".......info.x = %f\n", info.x);
    // 加速度方向为反向
    info.dx = (ak80info.speed[1] + ak80info.speed[0]) / 2 * wheel_radius;

    ROS_INFO("info.dx = %f\n", info.dx);

    // info.theta=-pitch+PI/2;
    info.theta = pitch;

    ROS_INFO("theta in IMU pitch......... %f\n", pitch);
    ROS_INFO("info.theta......... %f\n", info.theta);

    // 根据建模的方向前进的方向为正
    info.dtheta = dtheta;
    info.delta_yaw = yaw;
    info.ddelta_yaw = ddelta_yaw;

    // myimuinfo.x = v(0);
    // // myimuinfo.x=yaw_count;
    // myimuinfo.y = v(1);
    // myimuinfo.z = v(2);



    ROS_INFO_STREAM_THROTTLE(5, "\033[46;37m multi_info_pub \033[0m");

    // 发布信息
    multi_msg_pub.publish(info);
    myimuinfo_pub.publish(myimuinfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
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

void imu_Callback(const sensor_msgs::ImuConstPtr &imu_data)
{
  // float x,y,z,w;
  Eigen::VectorXd data(4);
  data << imu_data->orientation.w, imu_data->orientation.x, imu_data->orientation.y, imu_data->orientation.z;
  v = EulerianAngle(data);

  tf::Quaternion my_quaternion(imu_data->orientation.x,
                               imu_data->orientation.y,
                               imu_data->orientation.z,
                               imu_data->orientation.w);
  tf::Matrix3x3 m(my_quaternion);
  m.getRPY(roll, pitch, yaw);
  // *计算旋转的圈数
  current_yaw = yaw;

  if (previous_yaw - yaw > 6.1)
  {
    yaw_count++;
  }
  else if (yaw - previous_yaw > 6.1)
  {
    yaw_count--;
  }
  previous_yaw = yaw;
  yaw = 6.26 * yaw_count + yaw;

  dtheta = imu_data->angular_velocity.y;
  ddelta_yaw = imu_data->angular_velocity.z;

    myimuinfo.x=roll;
    myimuinfo.y=imu_data->angular_velocity.x;
    myimuinfo.z=7;

  // dx=imu_data->linear_acceleration
  ROS_INFO("This is the roll pitch yaw = %f,%f,%f\n", roll, pitch, yaw);
  // ROS_INFO_STREAM_ONCE("imu info successful");
  ROS_INFO_STREAM_THROTTLE(10, "\033[45;37m imucallback info successful \033[0m");
}

void ak80_Callback(const multi_msgs::ak80_info::ConstPtr &motor_ak80_info)
{

  ROS_INFO_STREAM_THROTTLE(10, "\033[45;37m ak80callnack info successful \033[0m");

  for (int i = 0; i < 2; i++)
  {
    ak80info.pos[i] = motor_ak80_info->pos[i];
    ak80info.speed[i] = motor_ak80_info->speed[i];
    ak80info.tau[i] = motor_ak80_info->tau[i];
    ak80info.tem[i] = motor_ak80_info->tem[i];
    ROS_INFO("ak80info.pos[%d] = %f\n", i, motor_ak80_info->pos[i]);
    ROS_INFO("ak80info.speed[%d] = %f\n", i, motor_ak80_info->speed[i]);
    ROS_INFO("ak80info.tau[%d] = %f\n", i, motor_ak80_info->tau[i]);
    // flag=false;
  }
}
