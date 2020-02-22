#include "sensors/imu.h"

namespace sensors
{
Imu::Imu()
{
    //Set Gyro Parameters
    _omega = Eigen::Vector3d::Zero();
    double sigma_gx, sigma_gy, sigma_gz;
    nh_.param<double>()
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