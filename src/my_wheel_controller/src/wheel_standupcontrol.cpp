/*
 * @Author: yuquan xu
 * @Date: 2023-02-10 10:43:52
 * @Last Modified by: mikey.zhaopeng
 * @Last Modified time: 2023-02-10 14:09:59
 */

// 多传感器，将多个信号订阅到一个节点并发布
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <multi_msgs/multi_info.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const sensor_msgs::ImuConstPtr &imu_data, const sensor_msgs::JointStateConstPtr &joint_states)
{
  // Solve all of perception here...
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  // message_filters::Subscriber<Image> image1_sub(nh, "image1", 1);
  // message_filters::Subscriber<Image> image2_sub(nh, "image2", 1);
  // ros::Subscriber sub = node.subscribe(imu_topic.c_str(), 10, &ImuCallback);
  // 订阅IMU的信息，整合到multi_msg
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu", 1);

  // 订阅joint_states的信息，整合到multi_msg,用于仿真验证
  message_filters::Subscriber<sensor_msgs::JointState> joint_states_sub(nh, "/joint_states", 1);

  // 发布订阅的整合信息
  ros::Publisher multi_msg_pub = nh.advertise<multi_msgs::multi_info>("/multi_info", 1000);

  typedef sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::JointState> MySyncPolicy;
  //   typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  //   boost::shared_ptr<Sync> sync_;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub, joint_states_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}