#include "my_gazebo_controller/gazebo_jump_controller.h"

using namespace Jump_controller;

int main(int argc, char **argv) {
  ros::init(argc, argv, "control_node");
  ros::NodeHandle nodeHandle("~");

  Jump_controller::jump_controller Jump_controller(nodeHandle);
  Jump_controller.receveMessage();

  // ros::Rate loop_rate(100);

  while (ros::ok()) {
    /*
    control logic
    */
    // clang-format off

//这部分代码不会被clang-format格式化

    // clang-format on
    Jump_controller.publish();
  }

  ros::spin();
  return 0;
}