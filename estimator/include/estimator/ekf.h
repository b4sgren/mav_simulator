#ifndef KALMANFILTER
#define KALMANFILTER

#include <ros/ros.h>
#include "mav_msgs/State.h"

namespace est
{
  class EKF
  {
  public:
    EKF();
    ~EKF();

  private:
    void stateCallback(const mav_msgs::StateConstPtr &msg);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_p;

    ros::Subscriber state_sub;
    ros::Publisher estState_pub;
  };
}
#endif
