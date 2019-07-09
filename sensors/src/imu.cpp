#include "sensors/imu.h"
#include <cmath>
#include <random>

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
       forces_ = Eigen::Matrix<double, 6, 1>::Zero();
       Omega_ = Eigen::Vector3d::Zero();
       A_ypr_ = Eigen::Vector3d::Zero();
       
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
        Omega_(0) = msg->p;
        Omega_(1) = msg->q;
        Omega_(2) = msg->r;
        A_ypr_(0) = msg->phi;
        A_ypr_(1) = msg->theta;
        A_ypr_(2) = msg->psi;
    }

    void IMU::forceCallback(const geometry_msgs::WrenchConstPtr& msg)
    {
        forces_(F) = msg->force.x;
        forces_(F+1) = msg->force.y;
        forces_(F+2) = msg->force.z;
        forces_(M) = msg->torque.x;
        forces_(M+1) = msg->torque.y;
        forces_(M+2) = msg->torque.z;
    }

    void IMU::generateReading()
    {
        Eigen::Vector3d gyro_bias;
        gyro_bias << gyro_bias_x_, gyro_bias_y_, gyro_bias_z_;

        Eigen::Vector3d accel_bias;
        accel_bias << accel_bias_x_, accel_bias_y_, accel_bias_z_;

        Eigen::Vector3d g_vec;
        g_vec << sin(A_ypr_(1)), -cos(A_ypr_(1)) * sin(A_ypr_(0)), -cos(A_ypr_(1)) * cos(A_ypr_(0));
        
        Eigen::Vector3d gyro_reading = Omega_ + gyro_bias + generateNoise() * stddev_g_;

        Eigen::Vector3d accel_reading = forces_.segment<3>(0)/mass_ + accel_bias + g_vec * g_ + generateNoise() * stddev_a_;

    }

    Eigen::Vector3d IMU::generateNoise()
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
        nh_.param<double>("gravity", g_, 9.81);
    }
}