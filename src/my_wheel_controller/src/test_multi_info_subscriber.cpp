/*
 * @Author: yuquan xu
 * @Date: 2023-02-10 10:43:52
 * @Last Modified by: yuquan xu
 * @Last Modified time: 2023-03-17 16:44:00
 */

// 多传感器，将多个信号订阅到一个节点并发布
#include "tf/transform_datatypes.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <multi_msgs/multi_info.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const sensor_msgs::ImuConstPtr &imu_data, const sensor_msgs::JointStateConstPtr &joint_states, ros::Publisher *multi_msg_pub)
{
  multi_msgs::multi_info info;
  // left_wheel_controller,wheel1_joint,站立轮子
  double x1 = joint_states->position[4];
  double x2 = joint_states->position[5];
  // right_wheel_controller,wheel2_joint,站立轮子
  info.x = 0.5 * (x1 + x2);
  double x3 = joint_states->velocity[4];
  double x4 = joint_states->velocity[5];
  info.dx = 0.5 * (x3 + x4);

  double roll, pitch, yaw;

  tf::Quaternion my_quaternion(imu_data->orientation.x,
                               imu_data->orientation.y,
                               imu_data->orientation.z,
                               imu_data->orientation.w);
  tf::Matrix3x3 m(my_quaternion);
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("roll = %f,pitch = %f,yaw = %f \n,", roll, pitch, yaw);

  double x = imu_data->orientation.x;
  double y = imu_data->orientation.y;
  double z = imu_data->orientation.z;
  double w = imu_data->orientation.w;
  // info.theta= - math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2));
  // info.delta = - math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2));
  // info.delta_yaw  = -imu_data.angular_velocity.x;
  // info.ddelta_yaw = -imu_data.angular_velocity.z;
  info.theta = pitch;
  // 加速度为何取负？？？
  info.dtheta = -imu_data->angular_velocity.x;

  info.delta_yaw = yaw;
  info.ddelta_yaw = -imu_data->angular_velocity.y;

  multi_msg_pub->publish(info);

  ROS_INFO("Synchronization successful");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  // message_filters::Subscriber<Image> image1_sub(nh, "image1", 1);
  // message_filters::Subscriber<Image> image2_sub(nh, "image2", 1);
  // ros::Subscriber sub = node.subscribe(imu_topic.c_str(), 10, &ImuCallback);
  // 订阅IMU的信息，整合到multi_msg,仿真的话题为/myrobot/sim_imu，实际的imu为/imu
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu", 1);

  // 订阅joint_states的信息，整合到multi_msg,用于仿真验证//gazebo中的话题是/myrobot/joint_states，实际的需要读取
  message_filters::Subscriber<sensor_msgs::JointState> joint_states_sub(nh, "/myrobot/joint_states", 1);

  // 发布订阅的整合信息
  ros::Publisher multi_msg_pub = nh.advertise<multi_msgs::multi_info>("/multi_info_sub", 1000);

  ROS_INFO("starting........multi_msg_pub............. \n");

  typedef sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::JointState> MySyncPolicy;
  // typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  // boost::shared_ptr<Sync> sync_;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub, joint_states_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, &multi_msg_pub));
  //&SubscribeAndPublish::
  // sync.registerCallback(boost::bind(&SubscribeAndPublish::callback, _1, _2));
  ros::spin();

  return 0;
}