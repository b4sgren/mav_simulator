#include "dynamics/mav_dynamics.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mav_dynamics");
  ros::NodeHandle nh;

  dyn::Dynamics mav;
  mav.run();
  return 0;
}
