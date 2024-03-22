#include "my_lqr_controller/leg_kinematics.h"
#include "ros/ros.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;
    LegKin leg_calcu = LegKin();
    for(int i=0;i<4;i++)
    {
    Eigen::Vector2d q =leg_calcu.InvKin(0,-0.23);
    Eigen::Matrix2d q2=leg_calcu.LegJac(0,0,0,-0.113);
    ROS_INFO("x:%f,y:%f",q(0),q(1));
    ROS_INFO("q20:%f,q21:%f",q2(0),q2(1));
    }


    ros::Rate loop_rate(500);
    ros::spin();
    loop_rate.sleep();
    return 0;
}
