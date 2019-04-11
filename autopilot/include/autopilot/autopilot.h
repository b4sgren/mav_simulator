#ifndef AUTOPILOT
#define AUTOPILOT

#include <ros/ros.h>
#include <Eigen/Core>
#include "autopilot/pid_controller.h"

namespace control
{
  class Autopilot
  {
  public:
    Autopilot();
    ~Autopilot();

  private:
    void estStateCallback(const dynamics::StateConstPtr& msg);
    // void commandsCallback(const autopilot::CommandsConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_p;

    ros::Publisher delta_pub;
    ros::Publisher commanded_state_pub;
    ros::Subscriber commands_sub; // TODO Add a commanded message
    ros::Subscriber est_state_sub;

    //Lateral Control loops
    PID_Controller roll_from_aileron;
    PID_Controller course_from_roll;
    // TODO Figure out Yaw damper

    //Longitudinal control loops
    PID_Controller pitch_from_elevator;
    PID_Controller altitude_from_pitch;
    PID_Controller airspeed_from_throttle;
  };
}

#endif
