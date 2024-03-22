#include "ros/ros.h"
#include "my_lqr_controller/lqr_control.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;
    my_control::Lqr_control nc = my_control::Lqr_control(nh);
    //Pid_control::pid_control nc = Pid_control::pid_control(nh);
    ros::Rate loop_rate(500);
    ros::spin();
    loop_rate.sleep();
    return 0;
}