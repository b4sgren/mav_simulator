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
  enum
  {
    POS = 0,
    VEL = 3,
    ATT = 6,
    OMEGA = 10
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
