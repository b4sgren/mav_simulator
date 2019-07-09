#ifndef IMU_SENSOR
#define IMU_SENSOR

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Wrench.h>
#include <dynamics/State.h>
#include <Eigen/Core>

namespace sensors
{
    class IMU
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        public:
        IMU();

        private:
        //Callbacks
        void stateCallback(const dynamics::StateConstPtr& msg);
        void forceCallback(const geometry_msgs::WrenchConstPtr& msg);

        //Other functions
        Eigen::Matrix<double, 6, 1> addNoise();
        void readParams();
        void run();
        void generateReading();

        //Ros variables
        ros::NodeHandle nh_;
        ros::NodeHandle nh_p_;
        
        ros::Subscriber state_sub_;
        ros::Subscriber force_sub_;
        ros::Publisher imu_pub_;

        //Other variables
        Eigen::Matrix3d accel_covar_;
        Eigen::Matrix3d gyro_covar_;
        double stddev_a_;
        double stddev_g_;
        double accel_bias_x_;
        double accel_bias_y_;
        double accel_bias_z_;
        double gyro_bias_x_;
        double gyro_bias_y_;
        double gyro_bias_z_;
        double mass_;

    };
}

#endif