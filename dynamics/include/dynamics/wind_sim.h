#ifndef WIND_SIM
#define WIND_SIM

#include <ros/ros.h>
#include <mav_msgs/Wind.h>
#include <mav_msgs/State.h>
#include <Eigen/Core>

namespace dyn
{
  class WindSim
  {
  public:
    WindSim();
    ~WindSim();

  private:
    void stateCallback(const mav_msgs::StateConstPtr &msg);
    void timerCallback(const ros::TimerEvent& event);

    double generate_random_double();
    Eigen::Vector3d getRandomVector();

    ros::NodeHandle nh_;
    ros::NodeHandle nh_p;

    ros::Subscriber state_sub;
    ros::Publisher wind_pub;

    ros::Timer timer_;
    Eigen::Matrix<double, 5, 5> A_;
    Eigen::Matrix<double, 5, 3> B_;
    Eigen::Matrix<double, 3, 5> C_;
    Eigen::Vector3d wind_ss_;
    Eigen::Matrix<double, 5, 1> wind_gust_;
    double Va_, Ts_;
    //Dryden Gust model Params
    double Lu_, Lv_, Lw_, sigma_u_, sigma_v_, sigma_w_;
  };
}

#endif
