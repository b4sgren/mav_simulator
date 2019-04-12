#include <ros/ros.h>
#include "estimator/ekf.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ekf");
  ros::NodeHandle nh;

  est::EKF ekf;

  ros::spin();
  return 0;
}
