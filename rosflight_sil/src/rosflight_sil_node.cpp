#include <ros/ros.h>
#include "rosflight_sil/rosflight_sil.h"

namespace rfs = rosflight_sil;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosflight_sil_node");
  rfs::ROSflightSIL rosflightsil;
  rosflightsil.Initialize();
  ros::spin();
  return 0;
}
