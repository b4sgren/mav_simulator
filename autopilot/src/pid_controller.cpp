#include "autopilot/pid_controller.h"

namespace control
{
  PID_Controller::PID_Controller(){}

  PID_Controller::PID_Controller(double kp, double kd, double ki, double sat_h, double sat_l)
  {
    kp_ = kp;
    kd_ = kd;
    ki_ = ki;
    sigma_= 0.05;
    limit_h_ = sat_h;
    limit_l_ = sat_l;
    integrator_ = 0.0;
    eprev_ = 0.0;
    yprev_ = 0.0;
    ydot_ = 0.0;
  }

  PID_Controller::~PID_Controller(){}

  double PID_Controller::update(double yref, double y, double dt, bool wrap_flag)
  {
    double error{yref - y};
    if(wrap_flag)
    {
      const double PI{3.14159265};
      while(error > PI)
        error -= (2 * PI);
      while(error <= -PI)
        error += (2 * PI);
    }
    integrateError(error, dt);
    differentiate(y, dt);

    double u_unsat{kp_ * error + ki_ * integrator_ - kd_ * ydot_};
    double u_sat{saturate(u_unsat)};

    if(ki_ != 0)
      antiWindUp(u_unsat, u_sat, dt);

    return u_sat;
  }

  double PID_Controller::updateWithRate(double yref, double y, double ydot, double dt)
  {
    double error{yref - y};
    integrateError(error, dt);

    double u_unsat{kp_ * error + ki_ * integrator_ - kd_ * ydot};
    double u_sat{saturate(u_unsat)};

    if(ki_ != 0)
      antiWindUp(u_unsat, u_sat, dt);

    return u_sat;
  }

  void PID_Controller::integrateError(double error, double dt)
  {
    integrator_ += dt/2.0 * (error + eprev_);
    eprev_ = error;
  }

  void PID_Controller::differentiate(double y, double dt)
  {
    double a1 = (2.0 * sigma_ - dt) / (2.0 * sigma_ + dt);
    double a2 = 2.0 / (2.0 * sigma_ + dt);

    ydot_ = a1 * ydot_ + a2 * (y - yprev_);
    yprev_ = y;
  }

  void PID_Controller::antiWindUp(double u_unsat, double u, double dt)
  {
    integrator_ += dt/ki_ * (u - u_unsat);
  }

  double PID_Controller::saturate(double u)
  {
    double usat;
    if(u >= limit_h_)
      usat = limit_h_;
    else if(u <= limit_l_)
      usat = limit_l_;
    else
      usat = u;
    return usat;
  }
}
