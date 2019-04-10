#ifndef WIND_SIM
#define WIND_SIM

#include <ros/ros.h>
#include <dynamics/Wind.h>
#include <Eigen/Core>

using namespace dyn
{
  class WindSim
  {
  public:
    WindSim();
    ~WindSim();

  private:
    void timerCallback(const ros::TimerEvent& event);
    void stateCallback(const dynamics::StateConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_p;

    ros::Subscriber state_sub;
    ros::Publisher wind_pub;

    ros::Timer timer_;
    Eigen::Matrix<double, 5, 5> A_;
    Eigen::Matrix<double, 5, 3> B_;
    Eigen::Matrix<double, 3, 5> C_;
    Eigen::Vector3d wind_ss_;
    Eigen::Vector3d wind_gust_;
    double Va_, Ts_;
    //Dryden Gust model Params
    double Lu_, Lv_, Lw_, sigma_u_, sigma_v_, sigma_w_;
  };
}

#endif
