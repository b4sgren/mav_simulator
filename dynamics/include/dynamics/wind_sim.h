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
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p;

    ros::Subscriber state_sub;
    ros::Publisher wind_pub;

    void timerCallback(const ros::TimerEvent& event);

    Eigen::Matrix<double, 5, 3> B;
    Eigen::Vector3d wind_ss;
    Eigen::Vector3d wind_gust;
    double Va_;
    //Dryden Gust model Params
    double Lu, Lv, LW, sigma_u, sigma_v, sigma_w;
  };
}

#endif
