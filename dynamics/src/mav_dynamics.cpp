#include "dynamics/mav_dynamics.h"

namespace dyn
{
Dynamics::Dynamics() : nh_(ros::NodeHandle()), nh_p_("~")
{
  Ts_ = 0.02;
  Va_ = 0.0;
  alpha_ = 0.0;
  beta_ = 0.0;

  x_<< 0.0, 0.0, -100.0, 25.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  wind_sub = nh_.subscribe("wind", 1, &Dynamics::windCallback, this);
  inputs_sub = nh_.subscribe("surface_commands", 1, &Dynamics::inputCallback, this);
  state_pub = nh_.advertise<dynamics::State>("state", 1);
}

Dynamics::~Dynamics(){}

void Dynamics::windCallback(const dynamics::WindConstPtr &msg)
{

}

void Dynamics::inputCallback(const dynamics::ControlInputsConstPtr &msg)
{

}
}
