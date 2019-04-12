#include "autopilot/autopilot.h"
#include <dynamics/ControlInputs.h>

namespace control
{
  Autopilot::Autopilot():  nh_(ros::NodeHandle()), nh_p("~")
  {
    //Write a parameter file
    double roll_kp, roll_kd, course_kp, course_ki, kp_yaw, tau_yaw;
    nh_p.param<double>("roll_kp", roll_kp, 0.0);
    nh_p.param<double>("roll_kd", roll_kd, 0.0);
    nh_p.param<double>("course_kp", course_kp, 0.0);
    nh_p.param<double>("course_ki", course_ki, 0.0);
    nh_p.param<double>("yaw_damper_tau_r", tau_yaw, 0.0);
    nh_p.param<double>("yaw_damper_kp", kp_yaw, 0.0);
    roll_from_aileron = PID_Controller(roll_kp, roll_kd, 0.0, radians(45), -radians(45));
    course_from_roll = PID_Controller(course_kp, 0, course_ki, radians(30), -radians(30));
    //TODO figure out yaw damper

    double pitch_kp, pitch_kd, alt_kp, alt_ki, throttle_kp, throttle_ki;
    nh_p.param<double>("pitch_kp", pitch_kp, 0.0);
    nh_p.param<double>("pitch_kd", pitch_kd, 0.0);
    nh_p.param<double>("altitude_kp", alt_kp, 0.0);
    nh_p.param<double>("altitude_ki", alt_ki, 0.0);
    nh_p.param<double>("throttle_kp", throttle_kp, 0.0);
    nh_p.param<double>("throttle_ki", throttle_ki, 0.0);
    pitch_from_elevator = PID_Controller(pitch_kp, pitch_kd, 0, radians(45), -radians(45));
    altitude_from_pitch = PID_Controller(alt_kp, 0, alt_ki, radians(30), -radians(30));
    airspeed_from_throttle = PID_Controller(throttle_kp, 0, throttle_ki, 1.0, 0.0);

    chi_ref_ = 0.0;
    nh_.param<double>("Va0", Va_ref_, 0.0);
    nh_.param<double>("h0", h_ref_, 0.0);
    phi_ff_ref_ = 0.0;

    delta_pub = nh_.advertise<dynamics::ControlInputs>("surface_commands", 1);
    commanded_state_pub = nh_.advertise<dynamics::State>("commanded_state", 1);
    commands_sub = nh_.subscribe("commands", 1, &Autopilot::commandsCallback, this); // TODO Add a commanded message
    est_state_sub = nh_.subscribe("estimated_states", 1, &Autopilot::estStateCallback, this);

    tprev_ = ros::Time::now().toSec();
  }

  Autopilot::~Autopilot(){}

  void Autopilot::estStateCallback(const dynamics::StateConstPtr &msg)
  {
    double t = ros::Time::now().toSec();
    double dt = t - tprev_;
    tprev_ = t;

    double phi_cmd = course_from_roll.update(chi_ref_, msg->chi, dt, true) + phi_ff_ref_;;
    double da = roll_from_aileron.updateWithRate(phi_cmd, msg->phi, msg->p, dt);
    //do yaw damper here
    double dr = 0.0;

    //longitudinal
    double theta_cmd = altitude_from_pitch.update(h_ref_, msg->h, dt, false);
    double de = pitch_from_elevator.updateWithRate(theta_cmd, msg->theta, msg->q, dt);
    double delt = airspeed_from_throttle.update(Va_ref_, msg->Va, dt, false);

    dynamics::ControlInputs deltas;
    deltas.header.stamp = ros::Time::now();
    deltas.da = da;
    deltas.dt = delt;
    deltas.dr = dr;
    deltas.de = de;
    delta_pub.publish(deltas);

    dynamics::State cmd_state;
    cmd_state.header.stamp = ros::Time::now();
    cmd_state.h = h_ref_;
    cmd_state.Va = Va_ref_;
    cmd_state.phi = phi_cmd;
    cmd_state.theta = theta_cmd;
    cmd_state.chi = chi_ref_;
    commanded_state_pub.publish(cmd_state);
  }

  void Autopilot::commandsCallback(const autopilot::CommandsConstPtr &msg)
  {
    chi_ref_ = msg->chi_cmd;
    Va_ref_= msg-> Va_cmd;
    h_ref_ = msg->h_cmd;
    phi_ff_ref_ = msg->phi_ff;
  }

  double Autopilot::radians(double deg)
  {
    double pi{3.14159625};
    return deg * 180.0/pi;
  }

}
