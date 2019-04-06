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
  std::cout << gamma6 << "\n";

  wind_sub = nh_.subscribe("wind", 1, &Dynamics::windCallback, this);
  inputs_sub = nh_.subscribe("surface_commands", 1, &Dynamics::inputCallback, this);
  state_pub = nh_.advertise<dynamics::State>("state", 1);
}

Dynamics::~Dynamics(){}

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
}

void Dynamics::windCallback(const dynamics::WindConstPtr &msg)
{
  //just update the wind here
}

void Dynamics::inputCallback(const dynamics::ControlInputsConstPtr &msg)
{
  //calc forces and moments
  //calc derivatives
  //update velocity data
  //update and publish state
}

StateVec Dynamics::derivatives(const StateVec& x)
{

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
}
