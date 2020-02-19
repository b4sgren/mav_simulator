#include "estimator/ekf.h"

namespace est
{
  EKF::EKF(): nh_(ros::NodeHandle()), nh_p("~")
  {
    state_sub = nh_.subscribe("true_states", 1, &EKF::stateCallback, this);
    estState_pub = nh_.advertise<mav_msgs::State>("estimated_states", 1);
  }

  EKF::~EKF(){}

  void EKF::stateCallback(const mav_msgs::StateConstPtr &msg)
  {
    mav_msgs::State estState;
    estState.header.stamp = ros::Time::now();
    estState.pn = msg->pn;
    estState.pe = msg->pe;
    estState.h = msg->h;
    estState.phi = msg->phi;
    estState.theta = msg->theta;
    estState.psi = msg->psi;
    estState.Va = msg->Va;
    estState.alpha = msg->alpha;
    estState.beta = msg->beta;
    estState.p = msg->p;
    estState.q = msg->q;
    estState.r = msg->r;
    estState.Vg = msg->Vg;
    estState.gamma = msg->gamma;
    estState.chi = msg->chi;
    estState.wn = msg->wn;
    estState.we = msg->we;
    estState.bx = 0.0;
    estState.by = 0.0;
    estState.bz = 0.0;

    estState_pub.publish(estState);
  }
}
