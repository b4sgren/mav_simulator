#include "dynamics/mav_dynamics.h"

#include <iostream>

namespace dyn
{
Dynamics::Dynamics() : nh_(ros::NodeHandle()), nh_p_("~")
{
  Ts_ = 0.005;
  alpha_ = 0.0;
  beta_ = 0.0;

  double de{-1.24778073e-1}, dt{6.76752115e-1}, da{1.83617600e-3}, dr{-3.0262668e-4}; //Initialize close to trim
  delta_r = dr;
  delta_a = da;
  delta_t = dt;
  delta_e = de;

  loadParams();
  forces_ = Eigen::Matrix<double, 6, 1>::Zero();
  chi_ = 0.0;
  flight_path_ = 0.0;

  windg_ = Eigen::Vector3d::Zero();
  wind_ss = Eigen::Vector3d::Zero();
  wind_ = Eigen::Vector3d::Zero();

  t0 = ros::Time::now().toSec();
  tprev = t0;

  wind_sub = nh_.subscribe("wind", 1, &Dynamics::windCallback, this);
  inputs_sub = nh_.subscribe("surface_commands", 1, &Dynamics::inputCallback, this);
  state_pub = nh_.advertise<mav_msgs::State>("true_states", 1);
}

Dynamics::~Dynamics(){}

void Dynamics::run()
{
    ros::Rate rate = 200;
    while(ros::ok())
    {
        //propogates and publishes the state
        propogateDynamics();
        ros::spinOnce();
        rate.sleep();
    }
}

void Dynamics::propogateDynamics()
{
    //calc forces and moments
    calculateForcesAndMoments();

    //4th order Runge-Kutta
    StateVec k1 = derivatives(x_);
    StateVec k2 = derivatives(x_ + Ts_/2.0 * k1);
    StateVec k3 = derivatives(x_ + Ts_/2.0 * k2);
    StateVec k4 = derivatives(x_ + Ts_ * k3);
    x_ += Ts_/6.0 * (k1 + 2*k2 + 2*k3 + k4);

    //normalize the quaternion
    Eigen::Vector4d e = x_.segment<4>(ATT);
    double norm_e = sqrt(e.transpose() * e);
    e = e / norm_e;
    x_.segment<4>(ATT) = e;

    //update and publish state
    updateVelocityData(windg_);
    calcGammaAndChi();
    mav_msgs::State state;
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
    state.Vg = sqrt(x_.segment<3>(VEL).transpose() * x_.segment<3>(VEL));
    state.gamma = flight_path_;
    state.chi = chi_;
    state.bx = 0.0;
    state.by = 0.0;
    state.bz = 0.0;
    state.header.stamp = ros::Time::now();

    state_pub.publish(state);
}

void Dynamics::windCallback(const mav_msgs::WindConstPtr &msg)
{
  wind_ss(0) = msg->wn;
  wind_ss(1) = msg->we;
  wind_ss(2) = msg->wd;
  windg_(0) = msg->gust_n;
  windg_(1) = msg->gust_e;
  windg_(2) = msg->gust_d;
  updateVelocityData(windg_);
}

void Dynamics::inputCallback(const mav_msgs::ControlInputsConstPtr &msg)
{
  delta_a = msg->da;
  delta_r = msg->dr;
  delta_t = msg->dt;
  delta_e = msg->de;
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

  double wp{w(0)}, wq{w(1)}, wr{w(2)}, vu{v(0)}, vv{v(1)}, vw{v(2)};
  double var1 = wr*vv - wq*vw;
  double var2 = wp*vw - wr*vu;
  double var3 = wq*vu - wp*vv;
  Eigen::Vector3d temp;
  temp << var1, var2, var3;
  xdot.segment<3>(VEL) = temp + 1.0/mass * f;

  xdot.segment<4>(ATT) = 0.5 * tools::skew(w) * e;

  xdot(OMEGA) = gamma1 * wp*wq - gamma2*wq*wr + gamma3 * moments(0) + gamma4*moments(2);
  xdot(OMEGA+1) = gamma5*wp*wr - gamma6*(wp*wp - wr*wr) + Jy*moments(1);
  xdot(OMEGA+2) = gamma7*wp*wq - gamma1*wq*wr + gamma4*moments(0) + gamma8*moments(2);

  return xdot;
}

void Dynamics::updateVelocityData(const Eigen::Vector3d& gust)
{
  Eigen::Matrix3d R_v2b = tools::Quaternion2Rotation(x_.segment<4>(ATT)).transpose();
  wind_ = R_v2b * wind_ss + gust;
  Eigen::Vector3d V = x_.segment<3>(VEL);

  //Compute Va
  Eigen::Vector3d Vr = V - wind_;
  Va_ = sqrt(Vr.transpose() * Vr);

  wind_ = R_v2b.transpose() * wind_;

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
  Eigen::Matrix3d R_b2v = tools::Quaternion2Rotation(x_.segment<4>(ATT));
  Eigen::Vector3d Vg = R_b2v * x_.segment<3>(VEL);

  flight_path_ = asin(-Vg(2)/(tools::norm(Vg)));

  Eigen::Vector3d Vgh = Vg * cos(flight_path_);
  Eigen::Vector3d e1{1, 0, 0};

  double temp = e1.transpose() * Vgh;
  chi_ = acos(temp/tools::norm(Vgh));
  if(Vgh(1) < 0)
    chi_ *= -1;
}

void Dynamics::calculateForcesAndMoments()
{
  //Calculate gravity
  Eigen::Matrix3d R_v2b = tools::Quaternion2Rotation(x_.segment<4>(ATT)).transpose();
  Eigen::Vector3d weight{0, 0, mass * g_};
  forces_.segment<3>(F) = R_v2b * weight;

  //update other forces
  calculateLongitudinalForces(delta_e);
  calculateLateralForces(delta_a, delta_r);
  calculateThrustForce(delta_t);
}

void Dynamics::calculateLongitudinalForces(double de)
{
  double c2V{c/(2 * Va_)};
  double q_bar{0.5 * rho * Va_*Va_ * S_wing};
  double e_negM = exp(-M_ * (alpha_ - alpha0));
  double e_posM = exp(M_ * (alpha_ + alpha0));
  double q = x_(OMEGA+1);

  double sigma_alpha = (1+e_negM + e_posM)/((1+e_negM)*(1+e_posM));
  double salpha = sin(alpha_);
  double calpha = cos(alpha_);

  double CLalpha = (1 - sigma_alpha)*(CL_0 + CL_alpha * alpha_) +
                   sigma_alpha * (2 * tools::sign(alpha_) * salpha*salpha * calpha);

  double F_lift = q_bar * (CLalpha + CL_q*c2V*q + CL_de*de);

  double temp = (CL_0 + CL_alpha*alpha_);
  double CDalpha = CD_p + (temp*temp)/(PI * e * AR);
  double F_drag = q_bar * (CDalpha + CD_q*c2V*q + CD_de*de);

  Eigen::Matrix2d R_s2b;
  R_s2b << calpha, -salpha, salpha, calpha;
  Eigen::Vector2d fxz;
  fxz << -F_drag, -F_lift;
  fxz = R_s2b * fxz;

  double m = q_bar * c * (Cm_0 + Cm_alpha*alpha_ + Cm_q*c2V*q + Cm_de*de);

  forces_(F) += fxz(0);
  forces_(F+2) += fxz(1);
  forces_(M+1) = m;
}

void Dynamics::calculateLateralForces(double da, double dr)
{
  double p{x_(OMEGA)}, r{x_(OMEGA+2)};

  double b2V = b/(2*Va_);
  double q_bar = 0.5 * rho * Va_*Va_ * S_wing;

  double fy = q_bar * (CY_0 + CY_beta*beta_ + CY_p*b2V*p + CY_r*b2V*r +
               CY_da*da + CY_dr*dr);

  double l = q_bar * b * (Cell_0 + Cell_beta*beta_ + Cell_p*b2V*p +
              Cell_r*b2V*r + Cell_da*da + Cell_dr*dr);

  double n = q_bar * b * (Cn_0 + Cn_beta*beta_ + Cn_p*b2V*p + Cn_r*b2V*r +
              Cn_da*da + Cn_dr*dr);

  forces_(M) = l;
  forces_(M+2) = n;
  forces_(F+1) += fy;
}

void Dynamics::calculateThrustForce(double dt)
{
  double V_in = V_max * dt;

  double a = (rho * pow(D_prop, 5))/pow(2*PI, 2) * C_Q0;
  double b2 = (rho * pow(D_prop, 4) * C_Q1 * Va_)/(2*PI) + (KQ*KQ)/R_motor;
  double c2 = rho * D_prop*D_prop*D_prop * C_Q2 * Va_*Va_ - (KQ*V_in)/R_motor + KQ * i0;

  double Omega_op = (-b2 + sqrt(b2*b2 - 4*a*c2))/(2*a);
  double J_op = (2 * PI * Va_)/(Omega_op * D_prop);

  double CT = C_T2 * (J_op*J_op) + C_T1*J_op + C_T0;
//  double CQ = C_Q2*(J_op*J_op) + C_Q2*J_op + C_Q0;

  double l = KQ * (1.0/R_motor * (V_in - KQ*Omega_op) - i0);
  double f = CT * (rho * (Omega_op * Omega_op) * pow(D_prop, 4))/(4*PI*PI);

  forces_(F) += f;
  forces_(M) -= l;
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
  nh_.param<double>("AR", AR, 0.0);
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
  nh_.param<double>("b", b, 0.0);
  nh_.param<double>("C_Y_0", CY_0, 0.0);
  nh_.param<double>("C_Y_beta", CY_beta, 0.0);
  nh_.param<double>("C_Y_p", CY_p, 0.0);
  nh_.param<double>("C_Y_r", CY_r, 0.0);
  nh_.param<double>("C_Y_delta_a", CY_da, 0.0);
  nh_.param<double>("C_Y_delta_r", CY_dr, 0.0);
  nh_.param<double>("C_ell_0", Cell_0, 0.0);
  nh_.param<double>("C_ell_beta", Cell_beta, 0.0);
  nh_.param<double>("C_ell_p", Cell_p, 0.0);
  nh_.param<double>("C_ell_r", Cell_r, 0.0);
  nh_.param<double>("C_ell_delta_a", Cell_da, 0.0);
  nh_.param<double>("C_ell_delta_r", Cell_dr, 0.0);
  nh_.param<double>("C_n_0", Cn_0, 0.0);
  nh_.param<double>("C_n_beta", Cn_beta, 0.0);
  nh_.param<double>("C_n_p", Cn_p, 0.0);
  nh_.param<double>("C_n_r", Cn_r, 0.0);
  nh_.param<double>("C_n_delta_a", Cn_da, 0.0);
  nh_.param<double>("C_n_delta_r", Cn_dr, 0.0);
  nh_.param<double>("D_prop", D_prop, 0.0);
  nh_.param<double>("C_Q0", C_Q0, 0.0);
  nh_.param<double>("C_Q1", C_Q1, 0.0);
  nh_.param<double>("C_Q2", C_Q2, 0.0);
  nh_.param<double>("KQ", KQ, 0.0);
  nh_.param<double>("R_motor", R_motor, 0.0);
  nh_.param<double>("i0", i0, 0.0);
  nh_.param<double>("C_T2", C_T2, 0.0);
  nh_.param<double>("C_T1", C_T1, 0.0);
  nh_.param<double>("C_T0", C_T0, 0.0);
  nh_.param<double>("V_max", V_max, 0.0);
}
}
