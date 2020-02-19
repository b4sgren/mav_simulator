#include "autopilot/autopilot.h"
#include <mav_msgs/ControlInputs.h>

namespace control
{
  Autopilot::Autopilot():  nh_(ros::NodeHandle()), nh_p("~")
  {
     starting = true;
    //Write a parameter file
    double roll_kp, roll_kd, course_kp, course_ki, kp_yaw, tau_yaw;
    nh_.param<double>("roll_kp", roll_kp, 0.0);
    nh_.param<double>("roll_kd", roll_kd, 0.0);
    nh_.param<double>("course_kp", course_kp, 0.0);
    nh_.param<double>("course_ki", course_ki, 0.0);
    nh_.param<double>("yaw_damper_tau_r", tau_yaw, 0.0);
    nh_.param<double>("yaw_damper_kp", kp_yaw, 0.0);
    roll_from_aileron = PID_Controller(roll_kp, roll_kd, 0.0, radians(45), -radians(45));
    course_from_roll = PID_Controller(course_kp, 0, course_ki, radians(30), -radians(30));
    //Yaw damper: Note that these values are specific to the values given in the control_params.yaml file
    A_ = 0.6;
    B_ = 0.02;
    C_ = -9.5;
    D_ = 0.5;
    x_ = 0.0;

    double pitch_kp, pitch_kd, alt_kp, alt_ki, throttle_kp, throttle_ki;
    nh_.param<double>("pitch_kp", pitch_kp, 0.0);
    nh_.param<double>("pitch_kd", pitch_kd, 0.0);
    nh_.param<double>("altitude_kp", alt_kp, 0.0);
    nh_.param<double>("altitude_ki", alt_ki, 0.0);
    nh_.param<double>("throttle_kp", throttle_kp, 0.0);
    nh_.param<double>("throttle_ki", throttle_ki, 0.0);
    pitch_from_elevator = PID_Controller(pitch_kp, pitch_kd, 0, radians(45), -radians(45));
    altitude_from_pitch = PID_Controller(alt_kp, 0, alt_ki, radians(30), -radians(30));
    airspeed_from_throttle = PID_Controller(throttle_kp, 0, throttle_ki, 1.0, 0.0);

    chi_ref_ = radians(-110);
    Va_ref_ = 25.0;
    h_ref_ = 110.0;
    phi_ff_ref_ = 0.0;

    delta_pub = nh_.advertise<mav_msgs::ControlInputs>("surface_commands", 1);
    commanded_state_pub = nh_.advertise<mav_msgs::State>("commanded_states", 1);
    commands_sub = nh_.subscribe("commands", 1, &Autopilot::commandsCallback, this);
    est_state_sub = nh_.subscribe("estimated_states", 1, &Autopilot::estStateCallback, this);

    tprev_ = ros::Time::now().toSec();
  }

  Autopilot::~Autopilot(){}

  void Autopilot::estStateCallback(const mav_msgs::StateConstPtr &msg)
  {
    double t = ros::Time::now().toSec();
    double dt = t - tprev_;
    tprev_ = t;

    double phi_cmd = course_from_roll.update(chi_ref_, msg->chi, dt, true) + phi_ff_ref_;
    double da = roll_from_aileron.updateWithRate(phi_cmd, msg->phi, msg->p, dt);
    //do yaw damper here
    x_ = A_ * x_ + B_ * msg->r;
    double dr = C_ * x_ + D_ * msg->r;

    //longitudinal
    double theta_cmd = altitude_from_pitch.update(h_ref_, msg->h, dt, false);
    double de = pitch_from_elevator.updateWithRate(theta_cmd, msg->theta, msg->q, dt);
    double delt = airspeed_from_throttle.update(Va_ref_, msg->Va, dt, false);

    mav_msgs::ControlInputs deltas;
    deltas.header.stamp = ros::Time::now();
    deltas.da = da;
    deltas.dt = delt;
    deltas.dr = dr;
    deltas.de = de;
    delta_pub.publish(deltas);

    mav_msgs::State cmd_state;
    cmd_state.header.stamp = ros::Time::now();
    cmd_state.h = h_ref_;
    cmd_state.Va = Va_ref_;
    cmd_state.phi = phi_cmd;
    cmd_state.theta = theta_cmd;
    cmd_state.chi = chi_ref_;
    commanded_state_pub.publish(cmd_state);
  }

  void Autopilot::commandsCallback(const mav_msgs::CommandsConstPtr &msg)
  {
    chi_ref_ = radians(msg->chi_cmd);
    Va_ref_= msg-> Va_cmd;
    h_ref_ = msg->h_cmd;
    phi_ff_ref_ = msg->phi_ff;
  }

  double Autopilot::radians(double deg)
  {
    double pi{3.14159265};
    return deg * pi/180.0;
  }

}
