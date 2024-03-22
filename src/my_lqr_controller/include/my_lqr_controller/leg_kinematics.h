#ifndef LEG_KINEMATICS_H
#define LEG_KINEMATICS_H
#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
class LegKin
{
public:
  LegKin()
  {
    L1=0.09;
    L2=0.09;
    L3=0.18;
    L4=0.18;
    d=0.1;
  }

  Eigen::Vector2d ForKin(double q1,double q2);//单腿正运动学 q1 q2是电机位置
  Eigen::Vector2d InvKin(double xe,double ye);//单腿逆运动学 xe ye是末端位置
  Eigen::Matrix2d LegJac(double q1,double q2,double x,double y);//单腿雅克比矩阵
  Eigen::Vector4d CalPassiveJoint(double q1,double q2,double dq1,double dq2);

private:
  double L1,L2,L3,L4,d;
  const double PI = acos(-1.0);
};






#endif // LEG_KINEMATICS_H