#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include "rosflight_sil/ROSflightSimState.h"
#include <std_msgs/Int32MultiArray.h>
#include <rosflight.h>
#include "sil_board.h"
#include <mavlink/mavlink.h>

namespace rosflight_sil {

class ROSflightSIL
{
public:
  ROSflightSIL();
  void Initialize();
  void OnUpdate(const ros::TimerEvent&);
  ~ROSflightSIL();

private:
  void truthCallback(const rosflight_sil::ROSflightSimState &msg);

  ros::NodeHandle* nh_;
  ros::NodeHandle* nh_private_;

  SIL_Board board_;
  rosflight_firmware::Mavlink comm_;
  rosflight_firmware::ROSflight firmware_;

  std::string mav_type_;
  ros::Timer timer_;

  ros::Subscriber truth_NED_sub_;
  ros::Publisher motor_pwm_pub_;

  bool truth_received_;

  // Time Counters
  double start_time_sec_;
  uint64_t start_time_us_;
};

} // end namespace rosflight_sil
