#include "sensors/imu.h"

namespace sensors
{
    IMU::IMU(): nh_(ros::NodeHandle()), nh_p_("~")
    {
       readParams();

       double covar(stddev_a_ * stddev_a_);
       Eigen::Vector3d temp(covar, covar, covar);
       accel_covar_ = temp.asDiagonal();

       covar = stddev_g_ * stddev_g_;
       Eigen::Vector3d temp2(covar, covar, covar);
       gyro_covar_ = temp2.asDiagonal();
       
       state_sub_ = nh_.subscribe("true_states", 1, &IMU::stateCallback, this);
       force_sub_ = nh_.subscribe("forces", 1, &IMU::forceCallback, this);
       imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);
    }
    
    void IMU::run()
    {
        ros::Rate rate = 125;

        while(ros::ok())
        {
            generateReading();
            ros::spinOnce();
            rate.sleep();
        }
    }

    void IMU::stateCallback(const dynamics::StateConstPtr& msg)
    {

    }

    void IMU::forceCallback(const geometry_msgs::WrenchConstPtr& msg)
    {

    }

    void IMU::generateReading()
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
        nh_.param<double>("mass", mass_, 10.0);
    }
}