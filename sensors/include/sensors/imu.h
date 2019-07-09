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
        enum
        {
            F = 0,
            M = 3
        };

        IMU();

        private:
        //Callbacks
        void stateCallback(const dynamics::StateConstPtr& msg);
        void forceCallback(const geometry_msgs::WrenchConstPtr& msg);

        //Other functions
        Eigen::Vector3d generateNoise();
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
        Eigen::Matrix<double, 6, 1> forces_;
        Eigen::Vector3d Omega_;
        Eigen::Vector3d A_ypr_;
        double stddev_a_;
        double stddev_g_;
        double accel_bias_x_;
        double accel_bias_y_;
        double accel_bias_z_;
        double gyro_bias_x_;
        double gyro_bias_y_;
        double gyro_bias_z_;
        double mass_;
        double g_;

    };
}

#endif