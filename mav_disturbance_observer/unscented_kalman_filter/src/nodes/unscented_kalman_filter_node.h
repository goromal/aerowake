/*
 * Copyright 2017 Andrea Tagliabue, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or otherwise.
 *
 */

#ifndef UNSCENTED_KALMAN_FILTER_UNSCENTED_KALMAN_FILTER_NODE_H
#define UNSCENTED_KALMAN_FILTER_UNSCENTED_KALMAN_FILTER_NODE_H

#include <iostream>
#include <stdio.h>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <unscented_kalman_filter/UKFConfig.h>

#include <std_srvs/SetBool.h>

#include "unscented_kalman_filter/unscented_kalman_filter.h"

#include <geometry-utils-lib/xform.h>

namespace unscented_kalman_filter {

//names for the subscribed and published topics
static char EXTERNAL_FORCES_MOMENTS[] = "external_forces_moments";
static char UFK_ESTIMATED_ODOMETRY[] = "ukf_est_odometry";
static char EXTERNAL_FORCES_MOMENT_COVARIANCE[] = "covariance_est_forces_moment";

class UnscentedKalmanFilterNode
{
 public:
  UnscentedKalmanFilterNode(const ros::NodeHandle&, const ros::NodeHandle&);
  ~UnscentedKalmanFilterNode();

 private:
//  transforms::Quatd q_X_UAV_;
//  transforms::Xformd X_X_NED_;
//  transforms::Xformd X_UAV_XAV_;
//  transforms::Quatd q_UAV_XAV_;

  static const size_t kStateSize = 16;
  static const size_t kMeasurementSize = 12;

  enum ForceOffsetSource : bool {STORED = false, COMPUTED = true} offset_source_;

  UnscentedKalmanFilter unscented_kalman_filter_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::string namespace_;

  bool pub_arrow_;
  visualization_msgs::Marker marker_;
  ros::Publisher marker_pub_;

  //subscribed to current motor velocity measurment
  //and current attitude measurement
  ros::Subscriber motor_velocity_measure_sub_;
  ros::Subscriber odometry_sub_;

  //Publishes 3 topics:
  //odometry_estimate_pub_ contains states estimates from ukf
  //external_forces_moments_estimate_pub_ contains estimated external forces
  //(in world reference frame) and estimated external moment around z axis (in vehicle ref frame)
  //external_forces_moment_covariance_pub_: covariance of the previous message.
  ros::Publisher odometry_estimated_pub_;
  ros::Publisher external_forces_moments_estimate_pub_;
  ros::Publisher external_forces_moment_covariance_pub_;

  // Dynamic Reconfigure
  dynamic_reconfigure::Server<unscented_kalman_filter::UKFConfig> server_;

  // Service Server
  ros::ServiceServer offset_removal_service_server_;

  //callback functions for the subscribed topic
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
  void MotorVelocityMeasureCallback(const mav_msgs::Actuators& actuators_msg);
  void DynamicReconfigureCallback(unscented_kalman_filter::UKFConfig& config, uint32_t level);
  bool OffsetRemovalServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

};
}
#endif
