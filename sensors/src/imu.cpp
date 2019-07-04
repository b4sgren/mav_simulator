#include "sensors/imu.h"

namespace sensors
{
    IMU::IMU(): nh_(ros::NodeHandle()), nh_p_("~")
    {
       readParams();
    }

    void IMU::stateCallback(const dynamics::StateConstPtr& msg)
    {

    }

    Eigen::Matrix<double, 6, 1> IMU::addNoise()
    {

    }

    void IMU::readParams()
    {
        nh_.param<double>("accel_sigma", stddev_a_, 0.05);
        nh_.param<double>("accel_x_bias", accel_bias_x_, 0.0);
        nh_.param<double>("accel_y_bias", accel_bias_y_, 0.0);
        nh_.param<double>("accel_z_bias", accel_bias_z_, 0.0);

        nh_.param<double>("gyro_sigma", stddev_g_, 0.005);
        nh_.param<double>("gyro_x_bias", gyro_bias_x_, 0.0);
        nh_.param<double>("gyro_y_bias", gyro_bias_y_, 0.0);
        nh_.param<double>("gyro_z_bias", gyro_bias_z_, 0.0);
    }
}