#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include "tf/transform_datatypes.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_gazebo");
  ros::NodeHandle nh;
  ros::Publisher move_publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);

  gazebo_msgs::ModelState model_state_pub;

  model_state_pub.model_name = "whell_describe";

  ros::Rate loop_rate(200);

  model_state_pub.pose.position.x = 0.0;
  model_state_pub.pose.position.y = 0.0;
  model_state_pub.pose.position.z = 0.5;

  tf::Quaternion quat;
  quat.setRPY(0,0,0);
  model_state_pub.pose.orientation.x = quat.x();
  model_state_pub.pose.orientation.y = quat.y();
  model_state_pub.pose.orientation.z = quat.z();
  model_state_pub.pose.orientation.w = quat.w();

  model_state_pub.reference_frame = "world";

  while(ros::ok())
  {
     move_publisher.publish(model_state_pub);
     loop_rate.sleep();
     ros::spinOnce();
  }

  return 0;
}

