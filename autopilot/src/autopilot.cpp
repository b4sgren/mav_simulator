#include "autopilot/autopilot.h"

namespace control
{
  Autopilot::Autopilot(): nh_{ros::NodeHandle()}, nh_p{"~"}
  {
    //Write a parameter file
  }

  Autopilot::~Autopilot(){}

}
