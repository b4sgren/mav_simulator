#ifndef IMU_SENSOR
#define IMU_SENSOR

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <dynamics/State.h>

namespace sensors
{
    class IMU
    {
        public:
        IMU();

        private:
        //Callbacks
        void stateCallback(const dynamics::StateConstPtr& msg);

        //Other functions

        //Ros variables
        ros::NodeHandle nh_;
        ros::NodeHandle nh_p_;
        
        ros::Subscriber state_sub_;
        ros::Publisher imu_pub_;

        //Other variables
        double stddev_a_;
        double stddev_g_;
        double accel_bias_x_;
        double accel_bias_y_;
        double accel_bias_z_;
        double gyro_bias_x_;
        double gyro_bias_y_;
        double gyro_bias_z_;

    };
}

#endif