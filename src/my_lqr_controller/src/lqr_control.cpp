#include "my_lqr_controller/lqr_control.h"
#include "ros/ros.h"


    Eigen::VectorXf states(5);
    Eigen::VectorXf states_rot(3);
    Eigen::VectorXf U(2);
    Eigen::VectorXf U_rot(2);
    Eigen::MatrixXf K(2,5);
    Eigen::MatrixXf K_rot(2,3);
//定义LQR控制下的参数
    Eigen::VectorXf states_lqr(4);
    Eigen::VectorXf states_rot_lqr(2);
    Eigen::VectorXf U_lqr(2);
    Eigen::VectorXf U_rot_lqr(2);
    Eigen::MatrixXf K_lqr(2,4);
    Eigen::MatrixXf K_rot_lqr(2,2);
namespace my_control{
void Lqr_control::init(ros::NodeHandle &nh){


        left_and_right_motor_wheel = nh.advertise<std_msgs::Float64MultiArray>("/myrobot/wheel12_Effort_controller/command", 1000);
        left_motor_wheel=nh.advertise<std_msgs::Float64>("/myrobot/left_wheel_controller/command",1000);
        right_motor_wheel=nh.advertise<std_msgs::Float64>("/myrobot/right_wheel_controller/command",1000);
        odometry_subscriber = nh.subscribe("/ground_truth/state", 5, &Lqr_control::callback_odom, this);
        imu_subscriber = nh.subscribe("/myrobot/sim_imu", 5, &Lqr_control::callback_imu, this);
        reference_position_topic = nh.subscribe("/reference_position", 5, &Lqr_control::callback_reference_position, this);

        K_lqr << -0.0158, -0.05203, -6.7448, -0.6457,
                -0.0158, -0.05203, -6.7448, -0.6457;


        K_rot_lqr << 0.5, 0.1773,
                     -0.5, -0.1773;

        current_posx = 0;
        current_velx = 0;
        current_pitch_rate = 0.0;
        current_yaw_rate = 0.0;
        dt = 0;
        error = 0.0;
        pitch = 0.0;
        Pitch = 0.0;
        Yaw =0.0;
        rot_error = 0.0;
        yaw = 0.0;
        left_torque.data = 0;
        right_torque.data = 0;
        current_time = ros::Time::now().toSec();
        // ROS_INFO("current_time while cunstructing object = %f \n", current_time);
        ref = 0;
        rot_integral_error = 0.0;
        rot_error = 0.0;
        //states_lqr << integral_error, current_posx, current_velx, Pitch, current_pitch_rate;
        //            位移X，dotX,俯仰角度pitch,dotPitch
        states_lqr << current_posx,current_velx,Pitch, current_pitch_rate;
        states_rot_lqr << Yaw, current_yaw_rate;
        // ROS_INFO("These are the rotational states upon initiazation %f %f %f \n",   states_rot(0),
        //                                                                             states_rot(1),
        //                                                                             states_rot(2) );

        ROS_INFO("hello,init Lqr_control...");
}

void Lqr_control::callback_reference_position(const std_msgs::Float64::ConstPtr& msg){

        ref = msg->data;

        ROS_INFO("hello,callback_reference_position ...");
}

void Lqr_control::callback_odom(const nav_msgs::Odometry::ConstPtr& msg){

        current_posx = msg->pose.pose.position.y;

        current_velx = msg->twist.twist.linear.y;
        tf::Quaternion my_quaternion(msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(my_quaternion);
        m.getRPY(roll, pitch, yaw);
        //ROS_INFO("This is the yaw from odom = %f \n", yaw);

        ROS_INFO("hello,callback_odom ...");

}

void Lqr_control::callback_imu(const sensor_msgs::Imu::ConstPtr& msg){


/*ROS_INFO("Value of same current_time from imu callback = %f \n", current_time);
ROS_INFO("Time fetched right now from imu callback = %f \n", ros::Time::now().toSec());  */

        dt = ros::Time::now().toSec() - current_time;
        if(dt>1){
            dt = 0.000000;
        }
        ROS_INFO("dt = %f \n", dt);
        error = current_posx - ref;
        rot_error = yaw;// - 0.5;
        //ROS_INFO("rotational error = %f \n", rot_error);
        rot_integral_error = rot_integral_error + rot_error*dt;
        integral_error = integral_error + error*dt;

        current_pitch_rate = msg->angular_velocity.x;
        current_yaw_rate = msg->angular_velocity.z;

        Pitch = roll;
        Yaw = yaw;

        states_lqr << current_posx, current_velx, Pitch, current_pitch_rate;
        states_rot_lqr << Yaw, current_yaw_rate;

        U_lqr << -9*K_lqr*states_lqr;
        ROS_INFO("These are the torque values %f %f \n", U(0), U(1));
        U_rot_lqr << -0*K_rot_lqr*states_rot_lqr;
        ROS_INFO("These are the torque values for U_lqr part %f %f \n", U_lqr(0), U_lqr(1));
        ROS_INFO("These are the torque values for U_rot_lqr part %f %f \n", U_rot_lqr(0), U_rot_lqr(1));
        // left_torque.data = U_rot(0) + U(0);
        // right_torque.data = U_rot(1) + U(1);
        left_torque.data = U_lqr(0)-U_rot_lqr(0);
        right_torque.data = U_lqr(1)+U_rot_lqr(1);
        ROS_INFO("The torque values for left and right wheel: %f %f \n", left_torque.data, right_torque.data);

        // for (int i = 0; i <2; ++i){

        //         msg_ball_wheel.data.clear();
        //         msg_ball_wheel.data.push_back(left_torque.data);
        //         msg_ball_wheel.data.push_back(right_torque.data);

        //     }

        // left_and_right_motor_wheel.publish(msg_ball_wheel);

        //发布左右轮子的力矩
        left_motor_wheel.publish(left_torque);
        right_motor_wheel.publish(right_torque);

        current_time = ros::Time::now().toSec();

        //ROS_INFO("Time fetched right now at the end of imu callback = %f \n", ros::Time::now().toSec());

        ROS_INFO("hello,callback_imu ...");
}
}
//下面是PID控制的程序

namespace Pid_control{
void pid_control::init(ros::NodeHandle &nh){


        left_motor_wheel=nh.advertise<std_msgs::Float64>("/myrobot/left_wheel_controller/command",1000);
        right_motor_wheel=nh.advertise<std_msgs::Float64>("/myrobot/right_wheel_controller/command",1000);
        odometry_subscriber = nh.subscribe("/ground_truth/state", 50, &pid_control::callback_odom, this);
        imu_subscriber = nh.subscribe("/myrobot/sim_imu", 50, &pid_control::callback_imu, this);
        reference_position_topic = nh.subscribe("/reference_position", 50, &pid_control::callback_reference_position, this);

        current_posx = 0;
        current_velx = 0;
        current_pitch_rate = 0;
        current_yaw_rate = 0.0;
        current_pitch_rate_pid=0;
        dt = 0;
        error = 0.0;
        pitch = 0.0;
        Pitch = 0.0;
        rot_error = 0.0;
        yaw = 0.0;
        left_torque.data = 0;
        right_torque.data = 0;
        current_time = ros::Time::now().toSec();

        ref = 0;
        rot_integral_error = 0.0;
        rot_error = 0.0;
        Pid_out = 0.0;

        ROS_INFO("hello,pid control ...");
}

void pid_control::callback_reference_position(const std_msgs::Float64::ConstPtr& msg){

        ref = msg->data;

        // ROS_INFO("hello,callback_reference_position ...");
}

void pid_control::callback_odom(const nav_msgs::Odometry::ConstPtr& msg){

        current_posx = msg->pose.pose.position.x;

        current_velx = msg->twist.twist.linear.x;
        tf::Quaternion my_quaternion(msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(my_quaternion);
        m.getRPY(roll, pitch, yaw);
        // ROS_INFO("1.....This is the pitch from odom = %f \n", pitch);
        // ROS_INFO("2.....This is the roll from odom = %f \n", roll);
        // ROS_INFO("3.....This is the yaw from odom = %f \n", yaw);

}

void pid_control::callback_imu(const sensor_msgs::Imu::ConstPtr& msg){


        ROS_INFO("hello,callback_imu ...");
        dt = ros::Time::now().toSec() - current_time;
        if(dt>1){
            dt = 0.0000001;
        }

        error = yaw;
        rot_error = roll;
        // ROS_INFO("rotational error = %f \n", rot_error);
        rot_integral_error = rot_integral_error + rot_error*dt;
        // integral_error += error*dt;

        current_pitch_rate = msg->angular_velocity.x;
        current_yaw_rate = msg->angular_velocity.z;

        Pitch = roll;
        Yaw=yaw;
        error=0-Pitch;
        integral_error += error*dt;
        current_pitch_rate_pid= error-previous_error;





        Pid_pitch_out=46*Pitch+4*current_pitch_rate;
        //   Pid_pitch_out=-44*error+7*current_pitch_rate;
        //   Pid_pitch_out=-44*error+10*current_pitch_rate_pid;

        Pid_yaw_out=0*Yaw+0*integral_error+0*current_yaw_rate;

        ROS_INFO("Current values current_pitch_rate %f\n", current_pitch_rate);
        ROS_INFO("Current values current_yaw_rate %f\n", current_yaw_rate);

        U << Pid_pitch_out,Pid_pitch_out;
        ROS_INFO(".........These are the torque values %f %f \n", U(0), U(1));
        U_rot<< Pid_yaw_out,Pid_yaw_out;
        ROS_INFO(".........These are the Pid_yaw_out values %f %f \n", U_rot(0), U_rot(1));

        left_torque.data = U(0)-U_rot(0);
        right_torque.data = U(1)+U_rot(1);
        left_motor_wheel.publish(left_torque);
        right_motor_wheel.publish(right_torque);

        previous_error=error;

        current_time = ros::Time::now().toSec();


}
}