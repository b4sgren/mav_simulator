#include "dynamics/mav_dynamics.h"

#include <iostream>

namespace dyn
{
Dynamics::Dynamics() : nh_(ros::NodeHandle()), nh_p_("~")
{
  Ts_ = 0.02;
  Va_ = 0.0;
  alpha_ = 0.0;
  beta_ = 0.0;

  loadParams();
  forces_ = Eigen::Matrix<double, 6, 1>::Zero();

  wind_sub = nh_.subscribe("wind", 1, &Dynamics::windCallback, this);
  inputs_sub = nh_.subscribe("surface_commands", 1, &Dynamics::inputCallback, this);
  state_pub = nh_.advertise<dynamics::State>("state", 1);
}

Dynamics::~Dynamics(){}

void Dynamics::windCallback(const dynamics::WindConstPtr &msg)
{
  //just update the wind here
}

void Dynamics::inputCallback(const dynamics::ControlInputsConstPtr &msg)
{
  //calc forces and moments
  calculateForcesAndMoments(msg);
  //4th order Runge-Kutta
  StateVec k1 = derivatives(x_);
  StateVec k2 = derivatives(x_ + Ts_/2.0 * k1);
  StateVec k3 = derivatives(x_ + Ts_/2.0 * k2);
  StateVec k4 = derivatives(x_ + Ts_ * k3);
  x_ += Ts_/6.0 * (k1 + 2*k2 + 2*k3 + k4);
  //update velocity data
  //update and publish state
}

StateVec Dynamics::derivatives(const StateVec& x)
{
  Eigen::Vector3d p = x.segment<3>(POS);
  Eigen::Vector3d v = x.segment<3>(VEL);
  Eigen::Vector4d e = x.segment<4>(ATT);
  Eigen::Vector3d w = x.segment<3>(OMEGA);
  Eigen::Vector3d f = forces_.segment<3>(F);
  Eigen::Vector3d moments = forces_.segment<3>(M);

  Eigen::Matrix3d R_b2v = tools::Quaternion2Rotation(e);

  StateVec xdot;
  xdot.segment<3>(POS) = R_b2v * v;

  double var1 = w(2) * v(1) - w(1) * v(2);
  double var2 = w(0) * v(2) - w(2) * v(0);
  double var3 = w(2) * v(0) - w(0) * v(2);
  Eigen::Vector3d temp;
  temp << var1, var2, var3;
  xdot.segment<3>(VEL) = temp + 1.0/mass * f;

  xdot.segment<4>(ATT) = 0.5 * tools::skew(w) * e;

  xdot(OMEGA) = gamma1 * w(0) * w(1) - gamma2*w(1)*w(2) + gamma3 * moments(0) + gamma4*moments(2);
  xdot(OMEGA+1) = gamma5*w(0)*w(2) - gamma6*(w(0)*w(0) - w(2)*w(2)) + Jy*moments(1);
  xdot(OMEGA+2) = gamma7*w(0)*w(1) - gamma1*w(1)*w(2) + gamma4*moments(0) + gamma8*moments(2);

  return xdot;
}

void Dynamics::updateVelocityData()
{

}

void Dynamics::calculateForcesAndMoments(const dynamics::ControlInputsConstPtr &msg)
{

}

void Dynamics::calculateLongitudinalForces(double de)
{

}

void Dynamics::calculateLateralForces(double da, double dr)
{

}

void Dynamics::calculateThrustForce(double dt)
{

}

void Dynamics::loadParams()
{
  //Load state variables
  double pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r;
  nh_.param<double>("pn0", pn, 0.0);
  nh_.param<double>("pe0", pe, 0.0);
  nh_.param<double>("pd0", pd, 0.0);
  nh_.param<double>("u0", u, 0.0);
  nh_.param<double>("v0", v, 0.0);
  nh_.param<double>("w0", w, 0.0);
  nh_.param<double>("e0", e0, 0.0);
  nh_.param<double>("e1", e1, 0.0);
  nh_.param<double>("e2", e2, 0.0);
  nh_.param<double>("e3", e3, 0.0);
  nh_.param<double>("p0", p, 0.0);
  nh_.param<double>("q0", q, 0.0);
  nh_.param<double>("r0", r, 0.0);
  x_ << pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r;
  nh_.param<double>("Va0", Va_, 0.0);

  nh_.param<double>("mass", mass, 10.0);
  nh_.param<double>("gamma", gamma, 0.0);
  nh_.param<double>("gamma1", gamma1, 0.0);
  nh_.param<double>("gamma2", gamma2, 0.0);
  nh_.param<double>("gamma3", gamma3, 0.0);
  nh_.param<double>("gamma4", gamma4, 0.0);
  nh_.param<double>("gamma5", gamma5, 0.0);
  nh_.param<double>("gamma6", gamma6, 0.0);
  nh_.param<double>("gamma7", gamma7, 0.0);
  nh_.param<double>("gamma8", gamma8, 0.0);
  nh_.param<double>("Jy", Jy, 0.0);
}
}
