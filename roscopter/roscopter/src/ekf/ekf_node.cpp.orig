#include <ros/ros.h>
#include "ekf/ekf_ros.h"

int main(int argc, char** argv)
{
#ifdef RELATIVE
    ros::init(argc, argv, "relative_estimator");
#else
  ros::init(argc, argv, "estimator");
#endif

  roscopter::ekf::EKF_ROS estimator;
  estimator.initROS();

  ros::spin();

  return 0;
}

