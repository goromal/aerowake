#pragma GCC diagnostic ignored "-Wwrite-strings"

#include <sstream>
#include <stdint.h>
#include <stdio.h>

#include <eigen3/Eigen/Core>

#include "rosflight_sil/rosflight_sil.h"
#include "rosflight_sil/sil_board.h"

namespace rosflight_sil {

ROSflightSIL::ROSflightSIL() : nh_(nullptr),
    nh_private_(nullptr), comm_(board_), firmware_(board_, comm_), truth_received_(false),
    start_time_sec_(0.0)
{}

ROSflightSIL::~ROSflightSIL()
{
  if (nh_)
  {
    nh_->shutdown();
    delete nh_;
  }
  if (nh_private_)
  {
    nh_private_->shutdown();
    delete nh_private_;
  }
}

void ROSflightSIL::Initialize()
{
  nh_ = new ros::NodeHandle();
  nh_private_ = new ros::NodeHandle("~");
  timer_ = nh_->createTimer(ros::Duration(ros::Rate(1000)), &ROSflightSIL::OnUpdate, this);

  board_.ros_setup(nh_, nh_private_, mav_type_);

  truth_NED_sub_ = nh_->subscribe("uav_truth_NED", 1, &ROSflightSIL::truthCallback, this);
  motor_pwm_pub_ = nh_->advertise<std_msgs::Int32MultiArray>("uav_motor_pwm", 1);
}

void ROSflightSIL::truthCallback(const rosflight_sil::ROSflightSimState &msg)
{
  board_.set_truth(msg);
  if (!truth_received_)
  {
    firmware_.init();
    start_time_sec_ = msg.header.stamp.toSec();
    truth_received_ = true;
  }
}

void ROSflightSIL::OnUpdate(const ros::TimerEvent&)
{
  if (truth_received_) // Physics are turned on, so ready to start simulating ROSflight
  {
    // We run twice so that that functions that take place when we don't have new IMU data get run
    firmware_.run();
    firmware_.run();

    // Obtain and send pwm motor commands (max of 14 channels)
    int* pwm_outputs = board_.get_outputs();
//    std::cout << "GIVEN: " << pwm_outputs[0] << " " << pwm_outputs[1] << " " << pwm_outputs[2] << " " << pwm_outputs[3] << std::endl; // ----
    int* end = pwm_outputs + 14;
    std_msgs::Int32MultiArray pwm_output_msg;
    for (int *p = pwm_outputs; p != end; ++p)
        pwm_output_msg.data.push_back(*p);
    motor_pwm_pub_.publish(pwm_output_msg);
  }
}

} // end namespace rosflight_sil
