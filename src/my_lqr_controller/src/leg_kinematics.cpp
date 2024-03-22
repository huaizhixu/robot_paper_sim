/*
 * @Author: yuquan xu 
 * @Date: 2023-07-18 14:33:28 
 * @Last Modified by: yuquan xu
 * @Last Modified time: 2023-07-18 15:31:52
 */
//
// % 闭链五杆机构运动学正解M
// %                y  |    / E
// %                   |  /  L6
// %                   | / B
// %                   |/\
// %                   /  \
// %               L2 /|   \ L3
// %                 / |    \
// %              A /  |     \ C
// %                \  |     /
// %              L1 \ |    / L4
// %          ---------|---------->
// %                  O  L5 D      x
// %

#include "my_lqr_controller/leg_kinematics.h"


Eigen::Vector2d LegKin::ForKin(double q1,double q2)
{
  using namespace std;

  double c1,c2,s1,s2;
  c1=cos(q1);s1=sin(q1);
  c2=cos(q2);s2=sin(q2);

  double xc,xd,yc,yd;
  xc=d/2+L1*c1;
  yc=-L1*s1;
  xd=-d/2-L2*c2;
  yd=-L2*s2;

  Eigen::Vector2d Vcd = Eigen::Vector2d(xc-xd,yc-yd);

  double CD,sfai,cfai,caph,saph,aph;
  CD = Vcd.norm();
  sfai=(yc-yd)/CD;
  cfai=(xc-xd)/CD;

  caph=(L4*L4-L3*L3-CD*CD)/(-2*L3*CD);
  aph=acos(caph);
  saph=sin(aph);

  Eigen::Vector2d pos;
  pos(0) = xc-L3*(caph*cfai-saph*sfai);
  pos(1) = yc-L3*(saph*cfai+sfai*caph);
  ROS_INFO("ForKin");
  return pos;
}

Eigen::Vector2d LegKin::InvKin(double xe,double ye)
{
  Eigen::Vector2d Q;
  double a,b,c;

  a=d*L1-2*xe*L1;
  b=2*L1*ye;
  c=xe*xe+ye*ye+d*d/4+L1*L1-xe*d-L3*L3;

  Q(0)= 2*atan((b-sqrt(a*a+b*b-c*c))/(a+c))-PI;

  a=d*L2+2*xe*L2;
  b=2*ye*L2;
  c=xe*xe+ye*ye+d*d/4+L2*L2+xe*d-L4*L4;

  Q(1)=2*atan((b-sqrt(a*a+b*b-c*c))/(a+c))-PI;

  while(Q(0)>PI){Q(0)=Q(0)-PI*2;}
  while(Q(0)<-PI){Q(0)=Q(0)+PI*2;}
  while(Q(1)>PI){Q(1)=Q(1)-PI*2;}
  while(Q(1)<-PI){Q(1)=Q(1)+PI*2;}
  return Q;
}

Eigen::Matrix2d LegKin::LegJac(double q1,double q2,double x,double y)
{
  // J = -inv(A)*B from matlab
  Eigen::Matrix2d J;

  double c1,c2,s1,s2,s12;
  c1=cos(q1);s1=sin(q1);
  c2=cos(q2);s2=sin(q2);
  s12=sin(q1+q2);

  double fac=2*d*y+2*L1*x*s1-2*L2*x*s2+2*L1*L2*s12+L1*d*s1+L2*d*s2+2*L1*y*c1+2*L2*y*c2;
  J(0,0)=(L1*(y+L2*s2)*(2*y*c1-d*s1+2*x*s1))/fac;
  J(0,1)=(L2*(y+L1*s1)*(d*s2-2*y*c2+2*x*s2))/fac;
  J(1,0)=-(L1*(d+2*x+2*L2*c2)*(2*y*c1-d*s1+2*x*s1))/(2*fac);
  J(1,1)=(L2*(d-2*x+2*L1*c1)*(d*s2-2*y*c2+2*x*s2))/(2*fac);
  return J;
}

//返回向量前两个是角度，后两个是速度
Eigen::Vector4d LegKin::CalPassiveJoint(double q1,double q2,double dq1,double dq2)
{
  Eigen::Vector4d res;
  Eigen::Vector2d pos = ForKin(q1,q2);
  double q3,q4,dq3,dq4;
  double dot_q3_q1,dot_q3_q2,dot_q4_q1,dot_q4_q2;

  Eigen::Vector2d A(d/2,0);
  Eigen::Vector2d B(-d/2,0);

  double AE=sqrt((pos-A).transpose()*(pos-A));
  double BE=sqrt((pos-B).transpose()*(pos-B));

  q3=PI-acos((L1*L1+L3*L3-AE*AE)/(2*L1*L3));
  q4=PI-acos((L2*L2+L4*L4-BE*BE)/(2*L2*L4));

  //q3,q4对q1，q2的偏导数
  dot_q3_q1=-(L3*sin(q1+q2+q3+q4)+L1*sin(q1+q2+q4))/(L3*sin(q1+q2+q3+q4));
  dot_q3_q2=(L2*sin(q4))/(L3*sin(q1+q2+q3+q4));
  dot_q4_q1=(L1*sin(q3))/(L4*sin(q1+q2+q3+q4));
  dot_q4_q2=-(L4*sin(q1+q2+q3+q4)+L2*sin(q1+q2+q3))/(L4*sin(q1+q2+q3+q4));

  dq3=dot_q3_q1*dq1+dot_q3_q2*dq2;
  dq4=dot_q4_q1*dq1+dot_q4_q2*dq2;

  res<<q3,q4,dq3,dq4;

  return res;
}


