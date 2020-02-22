#include <ros/ros.h>
#include "sensors/imu.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh;

    sensors::Imu imu;
    ros::spin();

    return 0;
}