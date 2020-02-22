#ifndef IMU
#define IMU

#include <ros/ros.h>
#include <mav_msgs/State.h>
#include <Eigen/Core>

namespace sensors
{
    class Imu
    {
    public:
        enum
          {
            POS = 0,
            VEL = 3,
            ATT = 6,
            OMEGA = 10,
            F = 0,
            M = 3
          };
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Imu();
        ~Imu();

    protected:
        void stateCallback(const mav_msgs::StateConstPtr &msg);
        void timerCallback(const ros::TimerEvent &event);

        ros::NodeHandle _nh;
        ros::NodeHandle _nh_p;

        ros::Subscriber _state_sub;
        ros::Publisher _imu_pub;

        ros::Timer _timer;

        Eigen::Vector3d _omega;
        Eigen::Vector3d _gyro_bias;
        Eigen::Matrix3d _R_gyro;
        double _mass;
    };
}

#endif