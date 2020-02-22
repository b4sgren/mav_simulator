#include "sensors/imu.h"

namespace sensors
{
Imu::Imu()
{
}

Imu::~Imu()
{
}

void Imu::stateCallback(const mav_msgs::StateConstPtr &msg)
{
}

void Imu::timerCallback(const ros::TimerEvent &event)
{
}

}