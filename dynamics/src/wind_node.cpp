#include <ros/ros.h>
#include <dynamics/wind.h>

int main(int argc, char** argv)
{
  ros::init(aargc, argv, "wind");
  ros::NodeHandle nh;

  dyn::WindSim wind;

  ros::spin();

  return 0;
}
