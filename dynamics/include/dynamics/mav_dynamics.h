#ifndef MAV_DYNAMICS
#define MAV_DYNAMICS

#include <ros/ros.h>
#include <dynamics/State.h>
#include <dynamics/Wind.h>
#include <dynamics/ControlInputs.h>
#include <Eigen/Core>
#include <cmath>
#include "tools/rotations.h"

typedef Eigen::Matrix<double, 13, 1> StateVec;
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
    OMEGA = 10,
    F = 0,
    M = 3
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Dynamics();
  ~Dynamics();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_p_;

  //Subscribers
  ros::Subscriber wind_sub;
  ros::Subscriber inputs_sub;

  //Publishers
  ros::Publisher state_pub;

  //callbacks
  void windCallback(const dynamics::WindConstPtr &msg);
  void inputCallback(const dynamics::ControlInputsConstPtr &msg);

  //Other functions
  void updateVelocityData();
  void calculateForcesAndMoments(const dynamics::ControlInputsConstPtr &msg);
  void calculateLongitudinalForces(double de);
  void calculateLateralForces(double da, double dr);
  void calculateThrustForce(double dt);
  StateVec derivatives(const StateVec& x);
  void loadParams();

  double Ts_, Va_, alpha_, beta_;
  StateVec x_;
  Eigen::Vector3d wind_;
  Eigen::Matrix<double, 6, 1> forces_;

  //Parameters from the yaml file
  double mass, gamma, gamma1, gamma2, gamma3, gamma4, gamma5, gamma6, gamma7, gamma8;
  double Jy;


};
}

#endif
