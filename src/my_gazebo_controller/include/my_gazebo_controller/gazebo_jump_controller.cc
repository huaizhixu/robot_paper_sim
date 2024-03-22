#include "my_gazebo_controller/gazebo_jump_controller.h"

using namespace Jump_controller;

// 初始化
void jump_controller::init() {
  setlocale(LC_ALL, "");
  ROS_INFO("\033[45;37m=================日志样式===================\033[0m");
  // ROS_INFO_STREAM("\033[31m 红色字 \033[0m");
  // ROS_INFO_STREAM("\033[32m 绿色字 \033[0m");
  // ROS_INFO_STREAM("\033[33m 黄色字 \033[0m");
  // ROS_INFO_STREAM("\033[34m 蓝色字 \033[0m");
  // ROS_INFO_STREAM("\033[36m 天蓝字 \033[0m");
  // ROS_INFO_STREAM("\033[37m 白色字 \033[0m");
  // ROS_INFO_STREAM("\033[40;37m 黑底白字 \033[0m");
  // ROS_INFO_STREAM("\033[41;37m 红底白字 \033[0m");
  // ROS_INFO_STREAM("\033[42;37m 绿底白字 \033[0m");
  // ROS_INFO_STREAM("\033[47;30m 白底黑字 \033[0m");
  // ROS_INFO_STREAM("\033[43;37m 黄底白字 \033[0m");
  // ROS_INFO_STREAM("\033[44;37m 蓝底白字 \033[0m");
  // ROS_INFO_STREAM("\033[45;37m 紫底白字 \033[0m");
  // ROS_INFO_STREAM("\033[46;37m 天蓝底白字 \033[0m");

  // ROS_INFO_STREAM_THROTTLE(2, "INFO throttle message.");
  // ROS_INFO_STREAM_ONCE("Output only once");

  // 全部腿的运动学对象获取
  LegKin Leg_Calcu = LegKin();

  sim_jump_flag.data = 0;
  has_hight = 0;
  stop_ak_flag.data = 0;
  // msg_leg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  // msg_leg.layout.dim[0].label = "leg1423_Effort";
  // msg_leg.layout.dim[0].size = 4;
  // msg_leg.layout.dim[0].stride = 1;
  // msg_leg.layout.data_offset=0;
  // msg_leg.data.clear();
  msg_leg.data.resize(4);

  // msg_leg.data.push_back(0);
  // msg_leg.data.push_back(0);
  // msg_leg.data.push_back(0);
  // msg_leg.data.push_back(0);
  for (int i = 0; i < 8; i++) {
    pos[i] = 0;
    vel[i] = 0;
    eff[i] = 0;
  }
  std::cout << pos[0] << "  ,  " << pos[1] << std::endl;

  // Eigen::Vector2d q=Leg_Calcu.ForKin(pos[0],-pos[1]);
  Eigen::Vector2d q = Leg_Calcu.ForKin(0, 0);
  init_y = q(1);
  Current_Height = q(1) - init_y;
  std::cout << q << std::endl;
  ROS_INFO("current_hight....  :%lf", Current_Height);
}

jump_controller::jump_controller() { init(); }

jump_controller::jump_controller(ros::NodeHandle &nodehandle)
    : nodehandle_(nodehandle) {
  // 初始化
  init();
  // 订阅
  subscriber_[0] = nodehandle_.subscribe(
      "/myrobot/joint_states", 100, &jump_controller::JointStateCallback, this);
  subscriber_[1] = nodehandle_.subscribe(
      "/simulation_jump_flag", 100,
      &jump_controller::simulation_start_jumpCallback, this);
  // 发布
  publisher_[0] = nodehandle_.advertise<std_msgs::String>("test_topic", 100);
  publisher_[1] = nodehandle_.advertise<std_msgs::Float64MultiArray>(
      "/myrobot/leg1423_Effort_controller/command", 1000);
  publisher_[2] =
      nodehandle_.advertise<std_msgs::Int8>("/simulation_stop_ak_flag", 1000);

  signal(SIGINT, &MySigintHandler);
}

void jump_controller::JointStateCallback(
    const sensor_msgs::JointStateConstPtr &msg) {
  ROS_INFO_STREAM_THROTTLE(7, "\033[45;37m JointStateCallback \033[0m");
  for (int i = 0; i < 8; i++) {
    pos[i] = msg->position[i];
    vel[i] = msg->velocity[i];
    eff[i] = msg->effort[i];
    if (i < 4) {
      if (i & 1) {
        pos[i] = -pos[i] - 0.5;
        vel[i] = -vel[i] - 0.5;
        eff[i] = -eff[i] - 0.5;
      } else {
        pos[i] = pos[i] - 0.5;
        vel[i] = vel[i] - 0.5;
        eff[i] = eff[i] - 0.5;
      }
    }
  }

  for (int i = 0; i < 4; i++) {
    std::cout << pos[i] << "  pos[],  " << std::endl;
  }
}

// 发布消息
void jump_controller::publish() {
  Eigen::Vector2d q = Leg_Calcu.ForKin(pos[0], pos[1]);
  std::cout << "pos12...." << pos[0] << pos[1] << std::endl;
  Eigen::Vector2d q1 = Leg_Calcu.InvKin(q(0), q(1));
  Current_Height = q(1) - init_y;
  // "\033[41;37m msg_leg publish \033[0m"
  std::cout << "q...." << q << std::endl;
  std::cout << "q1...." << q1 << std::endl;
  ROS_INFO("\033[41;37m current_hight....\033[0m  :%lf", Current_Height);

  if (pos[0] >= 1.98) {
    has_hight = 1;
    stop_ak_flag.data = 1;
    publisher_[2].publish(stop_ak_flag);
  }
  // sim_jump_flag.data接收到开始跳跃的信号
  if (sim_jump_flag.data == 1 && has_hight == 0) {
    // msg_leg.data.clear();
    msg_leg.data.push_back(2);
    msg_leg.data.push_back(-2);
    msg_leg.data.push_back(-2);
    msg_leg.data.push_back(2);
    // ROS_INFO_STREAM("has_hight........" << has_hight);
    // ROS_INFO_STREAM("pos[0]........:" << pos[0]);
  }

  if (sim_jump_flag.data == 0 && has_hight == 1) {
    // msg_leg.data.clear();
    msg_leg.data.push_back(1);
    msg_leg.data.push_back(-1);
    msg_leg.data.push_back(-1);
    msg_leg.data.push_back(1);
  }
  publisher_[1].publish(msg_leg);
  // ROS_INFO("msg_leg publish");
  ROS_INFO_STREAM_THROTTLE(7, "\033[41;37m msg_leg publish \033[0m");

  // std::cout<<"--------"<<has_hight<<std::endl;
  ros::spinOnce();
  usleep(1000);
}

// 接收消息
void jump_controller::receveMessage() {
  ROS_INFO_STREAM_ONCE("receveMessage Output only once");
}

// 接收到开始跳跃的信号
void jump_controller::simulation_start_jumpCallback(
    const std_msgs::Int8::ConstPtr &msg) {
  ROS_INFO_STREAM_THROTTLE(7,
                           "\033[45;37m simulation_start_jumpCallback \033[0m");
  sim_jump_flag.data = msg->data;
}

// ctrl+c 退出ros
void jump_controller::MySigintHandler(int sig) {
  ROS_INFO("shutting down!");
  ros::shutdown();
  exit(0);
}

jump_controller::~jump_controller() { ROS_INFO("~jump_controller!"); }