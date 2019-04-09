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
  chi_ = 0.0;
  flight_path_ = 0.0;

  wind_sub = nh_.subscribe("wind", 1, &Dynamics::windCallback, this);
  inputs_sub = nh_.subscribe("surface_commands", 1, &Dynamics::inputCallback, this);
  state_pub = nh_.advertise<dynamics::State>("state", 1);
}

Dynamics::~Dynamics(){}

void Dynamics::windCallback(const dynamics::WindConstPtr &msg)
{
  //TODO Something is wrong here
  //update wind
  if(!wind_init)
  {
    wind_ << msg->wn, msg->we, msg->wd;
    wind_init = true;
  }
  Eigen::Vector3d gust;
  gust << msg->gust_n, msg->gust_e, msg->gust_d;
  //update velocity data
  updateVelocityData(gust);
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

  //update and publish state
  calcGammaAndChi();
  dynamics::State state;
  state.pn = x_(POS);
  state.pe = x_(POS+1);
  state.h = -x_(POS+2);
  Eigen::Vector3d euler = tools::Quaternion2Euler(x_.segment<4>(ATT));
  state.phi = euler(0);
  state.theta = euler(1);
  state.psi = euler(2);
  state.p = x_(OMEGA);
  state.q = x_(OMEGA+1);
  state.r = x_(OMEGA+2);
  state.Va = Va_;
  state.alpha = alpha_;
  state.beta = beta_;
  state.wn = wind_(0);
  state.we = wind_(1);
  state.Vg = x_.segment<3>(VEL).transpose() * x_.segment<3>(VEL);
  state.gamma = flight_path_;
  state.chi = chi_;

  state_pub.publish(state);
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

void Dynamics::updateVelocityData(const Eigen::Vector3d& gust)
{
  Eigen::Matrix3d R_b2v = tools::Quaternion2Rotation(x_.segment<4>(ATT));
  wind_ = R_b2v * wind_ + gust;
  Eigen::Vector3d V = x_.segment<3>(VEL);

  //Compute Va
  Eigen::Vector3d Vr = V - wind_;
  Va_ = sqrt(Vr.transpose() * Vr);

  //update angle of attack
  if(Vr(0) == 0)
    alpha_ = tools::sign(Vr(2)) * PI/2.0;
  else
    alpha_ = atan2(Vr(2), Vr(0));

  //update sideslip angle
  double temp = sqrt(Vr(0)*Vr(0) + Vr(2)*Vr(2));
  if(temp == 0)
    beta_ = tools::sign(Vr(1)) * PI/2.0;
  else
    beta_ = asin(Vr(1)/Va_);
}

void Dynamics::calcGammaAndChi()
{
  Eigen::Matrix3d R_v2b = tools::Quaternion2Rotation(x_.segment<4>(ATT));
  Eigen::Vector3d Vg = R_v2b * x_.segment<3>(VEL);

  flight_path_ = asin(-Vg(0)/(tools::norm(Vg)));

  Eigen::Vector3d Vgh = Vg * cos(flight_path_);
  Eigen::Vector3d e1{1, 0, 0};

  double temp = e1.transpose() * Vgh;
  chi_ = acos(temp/tools::norm(Vgh));
  if(Vgh(0) < 0)
    chi_ *= -1;
}

void Dynamics::calculateForcesAndMoments(const dynamics::ControlInputsConstPtr &msg)
{
  //Calculate gravity
  Eigen::Matrix3d R_v2b = tools::Quaternion2Rotation(x_.segment<4>(ATT)).transpose();
  Eigen::Vector3d weight{0, 0, mass * g_};
  forces_.segment<3>(F) = R_v2b * weight;

  //update other forces
  double de{msg->de}, dt{msg->dt}, da{msg->da}, dr{msg->dr};
  calculateLongitudinalForces(de);
  calculateLateralForces(da, dr);
  calculateThrustForce(dt);
}

void Dynamics::calculateLongitudinalForces(double de)
{
  double c2V{c/(2 * Va_)};
  double q_bar{0.5 * rho * Va_*Va_ * S_wing};
  double e_negM = exp(-M_ * (alpha_ - alpha0));
  double e_posM = exp(M_ * (alpha_ + alpha0));

  double sigma_alpha = (1+e_negM + e_posM)/((1+e_negM)*(1+e_posM));
  double salpha = sin(alpha_);
  double calpha = cos(alpha_);

  double CLalpha = (1 - sigma_alpha)*(CL_0 + CL_alpha) +
                   sigma_alpha * (2 * tools::sign(alpha_) * salpha*salpha * calpha);
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
  nh_.param<double>("gravity", g_, 9.81);

  nh_.param<double>("M", M_, 0.0);
  nh_.param<double>("rho", rho, 0.0);
  nh_.param<double>("S_wing", S_wing, 0.0);
  nh_.param<double>("c", c, 0.0);
  nh_.param<double>("alpha0", alpha0, 0.0);
  nh_.param<double>("e", e, 0.0);
  nh_.param<double>("C_L_0", CL_0, 0.0);
  nh_.param<double>("C_L_alpha", CL_alpha, 0.0);
  nh_.param<double>("C_L_q", CL_q, 0.0);
  nh_.param<double>("C_L_delta_e", CL_de, 0.0);
  nh_.param<double>("C_D_0", CD_0, 0.0);
  nh_.param<double>("C_D_p", CD_p, 0.0);
  nh_.param<double>("C_D_alpha", CD_alpha, 0.0);
  nh_.param<double>("C_D_q", CD_q, 0.0);
  nh_.param<double>("C_D_delta_e", CD_de, 0.0);
  nh_.param<double>("C_m_0", Cm_0, 0.0);
  nh_.param<double>("C_m_alpha", Cm_alpha, 0.0);
  nh_.param<double>("C_m_q", Cm_q, 0.0);
  nh_.param<double>("C_m_delta_e", Cm_de, 0.0);
}
}
