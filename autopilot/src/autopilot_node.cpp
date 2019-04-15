#include <ros/ros.h>
#include "autopilot/autopilot.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "autopilot");
  ros::NodeHandle nh;

  control::Autopilot controller;

  ros::spin();
  return 0;
}
