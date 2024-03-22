#include "my_wheel_controller/lqr_control.h"
#include <cmath>
#define PI 3.1415926
const double pi = acos(-1.0);

bool flage = true;
float target_x = 0.0;
float target_speed = 0.0;
float target_pitch = 0.0;
float target_yaw = 0.0;

float speed_to_x = 0.0;

const Eigen::Matrix<double, 2, 2> &GetK2Matrix()
{
  static const struct Once
  {
    Eigen::Matrix<double, 2, 2> K;

    Once()
    {
      K << -70.711, -37.734,
          -70.711, -37.734;
    }
  } once;
  return once.K;
}

Eigen::MatrixXf K1(2, 4);
Eigen::MatrixXf K2(2, 2);
Eigen::Matrix<double, 2, 2> K3;
Eigen::VectorXf U1(2);
Eigen::VectorXf U2(2);
Eigen::VectorXf state1(4);
Eigen::VectorXf state2(2);

void lqr_control::init(ros::NodeHandle &nh)
{

  //! 初始化k1,k2,开始参数，Q:13000,1300,7000,1700,1700; R300
  //*误差x+-0.045

  // K1 << -4.6547, -4.3059, -16.2785, -2.9936,
  //       -4.6547, -4.3059, -16.2785, -2.9936;

  // K2 << 1.6833, 0.3802,
  //      -1.6833, -0.3802;

  //! 初始化k1,k2,开始参数，Q:71000,1300,7000,1700,1700; R300
  //*误差x+-0.04

  // K1 << -10.8781, -7.4001, -21.3690, -3.8100,
  //       -10.8781, -7.4001, -21.3690, -3.8100;

  // K2 << 1.6833, 0.3802,
  //      -1.6833, -0.3802;
  //! 初始化k1,k2,开始参数，Q:99000,1300,7000,1700,1700; R300
  //*误差< x+-0.04

  // K1 << -12.8452, -8.3105, -22.8109, -4.0489,
  //       -12.8452, -8.3105, -22.8109, -4.0489;

  // K2 << 1.6833, 0.3802,
  //      -1.6833, -0.3802;

  //! 初始化k1,k2,开始参数，Q:99000,1300,7000,1700,7777; R300
  //*误差< x+-0.04  短腿的好的参数

  // K1 << -12.8452, -8.3105, -22.8109, -4.0489,
  //       -12.8452, -8.3105, -22.8109, -4.0489;

  // K2 << 3.6002, 0.5560,
  //       -3.6002, -0.5560;
  //*新腿的参数Q:9000,1100,7000,700,7777; R300
  // K1 << -3.8730, -3.5923, -13.7314, -2.2684,
  //       -3.8730, -3.5923, -13.7314, -2.2684;

  // K2 << 3.6002, 0.5560,
  //       -3.6002, -0.5560;
  //! 新腿的参数Q:9999,1777,7000,700,7777; R300
  // K1 << -5.4432, -4.5798, -15.4732, -2.5632,
  //       -5.4444, -4.5798, -15.4732, -2.5632;

  // K2 << 3.6002, 0.5560,
  //       -3.6002, -0.5560;
  //! 新腿的参数Q:33333,1777,7000,1713,7777; R300
  // K1 << -7.4535, -5.8769, -19.0563, -3.4299,
  //     -7.4535, -5.8769, -19.0563, -3.4299;

  // K2 << 3.6002, 0.5560,
  //     -3.6002, -0.5560;

// 缓冲不使用位移

  K1 << -7.4535, -5.8769, -19.0563, -3.4299,
      -7.4535, -5.8769, -19.0563, -3.4299;

  K2 << 1.6833, 0.3802,
       -1.6833, -0.3802;


  // 测试初始化的方法
  K3 = GetK2Matrix();
  // !统计时间
  // begin = ros::Time::now().toSec();
  // std::cout << "time.....begin...:" << begin << std::endl;
  // 订阅多传感器信息
  multi_info = nh.subscribe("/multi_info_pub", 1000, &lqr_control::multinfo_Callback, this);

  // 订阅目标值,期望位置
  sub_target = nh.subscribe("/multi_target", 1000, &lqr_control::target_Callback, this);
  // 站立过程保持平衡
  sub_stand_up_lqr = nh.subscribe("stand_up_lqr", 1000, &lqr_control::stand_up_lqr_Callback, this);
  // 发布力矩
  left_motor_wheel_id13 = nh.advertise<std_msgs::Float64>("motor_wheel_id13", 1000);
  right_motor_wheel_id14 = nh.advertise<std_msgs::Float64>("motor_wheel_id14", 1000);

  state1_pub = nh.advertise<geometry_msgs::Vector3>("my_state1_info", 1000);

  // 初始化参数
  // 站立标志
  stand_up_lqr_flag = 0;
}

void lqr_control::target_Callback(const multi_msgs::target_info::ConstPtr &info)
{
  ROS_INFO_STREAM_THROTTLE(10, "\033[45;37m target_Callback \033[0m");
  target_x = info->tar_x;
  target_pitch = info->tar_pitch;
  target_speed = info->tar_speed;
  target_yaw = info->tar_yaw;
}

void lqr_control::multinfo_Callback(const multi_msgs::multi_info::ConstPtr &info)
{
  // ROS_INFO_STREAM_ONCE("multinfo_Callback");
  ROS_INFO_STREAM_THROTTLE(10, "\033[45;37m multinfo_Callback \033[0m");

  float x = info->x;
  float dx = info->dx;
  float theta = info->theta;
  // ROS_INFO("This is the pitch = %f\n", theta);
  float dtheta = info->dtheta;
  float delta_yaw = info->delta_yaw;
  float ddelta_yaw = info->ddelta_yaw;

  // ROS_INFO("init_x......init_x=%f\n",init_x);

  // 计算时间
  // double now=ros::Time::now().toSec();
  // double dt=now-begin;//第一次不是间隔0.02；
  double dt = 0.002;

  // std::cout << "time.....dt...:" << dt<< std::endl;
  // my_state1_info.x=dt;

  // *速度要进行累计位移x

  speed_to_x += target_speed * dt;

  // std::cout <<"target_speed.....::"<< target_speed << std::endl;

  state1 << x - target_x - speed_to_x, dx - target_speed, theta, dtheta;

  // my_state1_info.x=state1(0);
  // std::cout <<"state1.....::"<< state1 << std::endl;

  state2 << delta_yaw - target_yaw, ddelta_yaw;

  // ROS_INFO("state1......x, theta=: %f ,%f\n",x,theta);
  // 调试曲线使用
  // my_state1_info.x=x;
  // my_state1_info.y=theta;
  // my_state1_info.z=dtheta;
  // std::cout <<"state1:"<< state1 << std::endl;
  // std::cout <<"state2:"<< state2 << std::endl;

  U1 << -K1 * state1;
  U2 << -K2 * state2;

  // std::cout <<"U1::"<< U1 << std::endl;
  // std::cout <<"U2::"<< U2 << std::endl;

  // *加上平衡和转向
  msg13.data = U1(0) - U2(0);
  msg14.data = U1(1) - U2(1);

  // 只是调试平衡的问题
  //  msg13.data = U1(0);
  //  msg14.data = U1(1);

  // 只是调试yaw
  //  msg13.data =  -U2(0);
  //  msg14.data =  -U2(1);

  // std::cout << U1(0) <<" U1(0).....!\n";
  // std::cout << U1(0) <<" U1(1).....!\n";

  // std::cout << U2(0) <<" U2(0).....!\n";

  // 限制力矩输出
  if (msg13.data > 10)
  {

    msg13.data = 10;
  }
  if (msg14.data > 10)
  {

    msg14.data = 10;
  }

  if (msg13.data > 0.2)
  {
    msg13.data = msg13.data - 0.11;
    // std::cout <<"msg13.data>0.2"<< std::endl;
  }
  else if (msg13.data < -0.2)
  {
    msg13.data = msg13.data + 0.1;
    // std::cout <<"msg13.data>0.2"<< std::endl;
  }
  // 站立过程保持平衡stand_up_lqr_flag
  if (!stand_up_lqr_flag)
  {
    msg13.data = 0.0;
    msg14.data = 0.0;
  }
  // }else if(stand_up_lqr_flag==1){

  //   }else if(stand_up_lqr_flag==8){
  //   msg13.data=0.0;
  //   msg14.data=0.0;
  // }

  my_state1_info.x = msg13.data;
  my_state1_info.y = msg14.data;
  my_state1_info.z = target_speed;

  // std::cout << msg13.data <<" msg13.data.........................!\n";

  left_motor_wheel_id13.publish(msg13);
  right_motor_wheel_id14.publish(msg14);
  state1_pub.publish(my_state1_info);

  // begin=now;

  ROS_INFO_STREAM_THROTTLE(7, "\033[46;37m motor_wheel_id_13_and_14_pub \033[0m");
}

void lqr_control::stand_up_lqr_Callback(const std_msgs::Int32::ConstPtr &msg)
{
  stand_up_lqr_flag = msg->data;
}
