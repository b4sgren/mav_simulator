#include "dynamics/wind_sim.h"
#include <cmath>
#include <random>

namespace dyn
{
  WindSim::WindSim(): nh_(ros::NodeHandle()), nh_p("~")
  {
    nh_.param<double>("Va0", Va_, 0.0);

    wind_ss_ << 3.0, 1.0, 0.0;
    wind_gust_ << 0.0, 0.0, 0.0, 0.0, 0.0;
    Ts_ = 0.02;

    //These should be in a parameter file
    Lu_ = 200.0;
    Lv_ = Lu_;
    Lw_ = 50.0;
    sigma_u_ = 2.12;
    sigma_v_ = 2.12;
    sigma_w_ = 1.4;

    A_ = Eigen::Matrix<double, 5, 5>::Zero();
    A_(2,1) = 1;
    A_(4,3) = 1;

    B_ << 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    C_ = Eigen::Matrix<double, 3, 5>::Zero();

    state_sub = nh_.subscribe("true_states", 1, &WindSim::stateCallback, this);
    wind_pub = nh_.advertise<mav_msgs::Wind>("wind", 1);

    timer_ = nh_.createTimer(ros::Duration(0.1), &WindSim::timerCallback, this);
  }

  WindSim::~WindSim(){}

 void WindSim::timerCallback(const ros::TimerEvent& event)
 {
   Eigen::Vector3d w = getRandomVector();
   wind_gust_ = wind_gust_ + Ts_ * (A_ * wind_gust_ + B_ * w);

   Eigen::Vector3d temp = C_ * wind_gust_;

   mav_msgs::Wind wind_msg;
   wind_msg.wn = wind_ss_(0);
   wind_msg.we = wind_ss_(1);
   wind_msg.wd = wind_ss_(2);
   wind_msg.gust_n = temp(0);
   wind_msg.gust_e = temp(1);
   wind_msg.gust_d = temp(2);
   wind_msg.header.stamp = ros::Time::now();

   wind_pub.publish(wind_msg);
 }

 void WindSim::stateCallback(const mav_msgs::StateConstPtr &msg)
 {
   Va_ = msg->Va;

   //recompute A and C matrices
   A_(0,0) = -Va_/Lu_;
   A_(1,1) = -2 * Va_/Lv_;
   A_(1,2) = -(Va_*Va_)/(Lv_*Lv_);
   A_(3,3) = -2 * Va_/Lw_;
   A_(3,4) = -(Va_*Va_)/(Lw_*Lw_);

   C_(0,0) = sigma_u_ * sqrt(2 * Va_/Lu_);
   C_(1,1) = sigma_v_ * sqrt(3 * Va_/Lv_);
   C_(1,2) = sqrt(pow(Va_/Lv_, 3));
   C_(2,3) = sigma_w_ * sqrt(3 * Va_/Lv_);
   C_(2,4) = sqrt(pow(Va_/Lw_, 3));
 }

 double WindSim::generate_random_double()
 {
   std::random_device rd;
   std::mt19937 gen(rd());
   std::normal_distribution<> dis(0.0, 1.0);
   return dis(gen);
 }

 Eigen::Vector3d WindSim::getRandomVector()
 {
   Eigen::Vector3d temp;
   temp << generate_random_double(), generate_random_double(), generate_random_double();

   return temp;
 }
}
