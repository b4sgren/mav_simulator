#ifndef KALMANFILTER
#define KALMANFILTER

#include <ros/ros.h>
#include "dynamics/State.h"

namespace est
{
  class EKF
  {
  public:
    EKF();
    ~EKF();

  private:
    void stateCallback(const dynamics::StateConstPtr &msg);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_p;

    ros::Subscriber state_sub;
    ros::Publisher estState_pub;
  };
}
#endif
