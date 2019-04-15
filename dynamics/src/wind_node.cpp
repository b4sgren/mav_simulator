#include <ros/ros.h>
#include "dynamics/wind_sim.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wind");
  ros::NodeHandle nh;

  dyn::WindSim wind;

  ros::spin();
  return 0;
}
