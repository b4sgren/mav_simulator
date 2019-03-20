#ifndef MAV_DYNAMICS
#define MAV_DYNAMICS

#include <ros/ros.h>
// #include <msg/State>
#include <Eigen/Core>
#include <cmath>

typedef Eigen::Matrix<double, 12, 1> StateVec;
namespace dyn
{

class Dynamics
{
public:
  Dynamics();
  ~Dynamics();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_p_;

  double Ts_, Va_, alpha_, beta_;
  StateVec x_;
  Eigen::Vector3d wind_;
  Eigen::Vector3d forces;


};
}

#endif
