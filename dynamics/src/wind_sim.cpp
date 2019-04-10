#include "dynamics/wind.h"
#include <cmath>

using namespace dyn
{
  WindSim::WindSim(): nh_(ros::NodeHandle()), nh_p("~")
  {
    nh_.param<double>("Va0", Va_, 0.0);

    wind_ss_ << 3.0, 1.0, 0.0;
    wind_gust_ << 0.0, 0.0, 0.0;

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
    C_ = Eigen::Matrox<double, 3, 5>::Zero();

    timer_ = nh_.createTimer(ros::Duration(0.1), &timerCallback);
  }

  WindSim::~WindSim(){}

 void WindSim::timerCallback(const ros::TimerEvent& event)
 {

 }

 void stateCallback(const dynamics::StateConstPtr& msg)
 {

 }
}
