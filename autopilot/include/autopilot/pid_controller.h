#ifndef PID
#define PID

#include <ros/ros.h>

namespace control
{
  class PID_Controller
  {
  public:
    PID_Controller(double kp, double kd, double ki, double sigma, double sat_l, double sat_h);
    ~PID_Controller();

  private:
    double update(double yref, double y, double dt, bool wrap_flag);
    double updateWithRate(double yref, double y, double ydot, double dt);
    void integrateError(double error, double dt);
    void differentiate(double y, double dt);
    void antiWindUp(double u_unsat, double u, double dt);
    double saturate(double u);

    double kp_, kd_, ki_, sigma_, limit_h_, limit_l_;
    double integrator_, eprev_, ydot_, yprev_;
    const double PI{3.14159625};

  };
}
#endif
