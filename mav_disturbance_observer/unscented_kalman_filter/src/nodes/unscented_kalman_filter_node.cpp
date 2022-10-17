/*
 * Copyright 2017 Andrea Tagliabue, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or otherwise.
 *
 */

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <geometry_msgs/WrenchStamped.h>

#include "unscented_kalman_filter_node.h"

namespace unscented_kalman_filter {

UnscentedKalmanFilterNode::UnscentedKalmanFilterNode(const ros::NodeHandle& nh,
                                                     const ros::NodeHandle& nh_private)
    : unscented_kalman_filter_(nh, nh_private), 
    nh_(nh),
    nh_private_(nh_private), 
    offset_source_(ForceOffsetSource::STORED)
{
//    X_X_NED_ = transforms::Xformd((Vector3d() << 0., 0., 0.).finished(), transforms::Quatd::from_euler(3.14159265, 0., 0.));
//    q_UAV_XAV_ = transforms::Quatd::from_euler(3.14159265, 0., 0.);
//    X_UAV_XAV_ = transforms::Xformd((Vector3d() << 0., 0., 0.).finished(), q_UAV_XAV_);
//    q_X_UAV_ = transforms::Quatd::Identity();

  // initialize subscription to motor velocity
  motor_velocity_measure_sub_ = nh_.subscribe(
      mav_msgs::default_topics::MOTOR_MEASUREMENT, 1,
      &UnscentedKalmanFilterNode::MotorVelocityMeasureCallback, this,
      ros::TransportHints().tcpNoDelay());

  // initialize suscription to odometry, which is the pose estimate from the "above" EKF
  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &UnscentedKalmanFilterNode::OdometryCallback, this,
                                ros::TransportHints().tcpNoDelay());

  // initialize publication of external forces and external moments
  external_forces_moments_estimate_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(
      EXTERNAL_FORCES_MOMENTS, 1);

  // initialize publication of states estimated by UKF
  odometry_estimated_pub_ = nh_.advertise<nav_msgs::Odometry>(
      unscented_kalman_filter::UFK_ESTIMATED_ODOMETRY, 1);

  // for diagnostic, initialize publication of force and torque covariance
  external_forces_moment_covariance_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(
      EXTERNAL_FORCES_MOMENT_COVARIANCE, 1);

  // Initializing Dynamic Reconfiguration
  dynamic_reconfigure::Server<unscented_kalman_filter::UKFConfig>::CallbackType f;
  f = boost::bind(&UnscentedKalmanFilterNode::DynamicReconfigureCallback, this, _1, _2);
  server_.setCallback(f);

  // Service server initialization
  offset_removal_service_server_ = nh_.advertiseService(
      "ukf_remove_offset", &UnscentedKalmanFilterNode::OffsetRemovalServiceCallback, this);

  pub_arrow_ = nh_private_.param("pub_arrow", false);
  if (pub_arrow_)
  {
      double arrow_scale = 0.1;
      marker_.header.frame_id = "UAV";
      marker_.id = 44;
      marker_.type = visualization_msgs::Marker::ARROW;
      marker_.action = visualization_msgs::Marker::ADD;
      marker_.pose = geometry_msgs::Pose();
      marker_.pose.position.x = 0.0;
      marker_.pose.position.y = 0.0;
      marker_.pose.position.z = 0.0;
      marker_.scale.x = arrow_scale;
      marker_.scale.y = arrow_scale;
      marker_.scale.z = arrow_scale;
      marker_.color.r = 0.0;
      marker_.color.g = 0.0;
      marker_.color.b = 0.0;
      marker_.color.a = 1.0;
      marker_pub_ = nh_.advertise<visualization_msgs::Marker>("Fext_marker", 1);
  }
}

UnscentedKalmanFilterNode::~UnscentedKalmanFilterNode()
{
}

void UnscentedKalmanFilterNode::DynamicReconfigureCallback(
    unscented_kalman_filter::UKFConfig &config, uint32_t level)
{
  ROS_INFO_STREAM("Unscented kalman filter: dynamic reconfigure request received.");
  
  // Tuning Q and R matrices
  Eigen::DiagonalMatrix<double, kStateSize> Q;
  Q.diagonal() << config.Q_position, config.Q_position, config.Q_position,
                  config.Q_velocity, config.Q_velocity, config.Q_velocity, 
                  config.Q_attitude, config.Q_attitude, config.Q_attitude, 
                  config.Q_angularvelocity, config.Q_angularvelocity, config.Q_angularvelocity, 
                  config.Q_external_FxFy, config.Q_external_FxFy, config.Q_external_Fz, 
                  config.Q_external_Myaw;
  unscented_kalman_filter_.setProcessNoiseCovariance(Q);
  
  Eigen::DiagonalMatrix<double, kMeasurementSize> R;
  R.diagonal() << config.R_position, config.R_position, config.R_position, 
                  config.R_velocity, config.R_velocity, config.R_velocity, 
                  config.R_attitude, config.R_attitude, config.R_attitude, 
                  config.R_angularvelocity, config.R_angularvelocity, config.R_angularvelocity;
  unscented_kalman_filter_.setMeasurementNoiseCovariance(R);
  
  // Unscented transformation params
  unscented_kalman_filter_.setAlphaBetaKappaMrpA(config.alpha, config.beta, config.kappa,
                                                 config.MRP_a);
  // force offset computation params
  unscented_kalman_filter_.setForceOffsetParams(config.force_offset_computation_time);

  // first argument: "second order dynamics" (takes into account aerodynamic forces)
  // last argument: avoids averaging quaternion algorithm and uses a more robust 
  // approximation
  unscented_kalman_filter_.setSettings(true, config.debug_mode, false);
}

void UnscentedKalmanFilterNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{

  ROS_INFO_ONCE("Unscented Kalman Filter received first odometry message.");

//  transforms::Xformd X_NED_UAV((Eigen::Vector3d() << odometry_msg->pose.pose.position.x,
//                                                     odometry_msg->pose.pose.position.y,
//                                                     odometry_msg->pose.pose.position.z).finished(),
//                               transforms::Quatd((Vector4d() << odometry_msg->pose.pose.orientation.w,
//                                                                odometry_msg->pose.pose.orientation.x,
//                                                                odometry_msg->pose.pose.orientation.y,
//                                                                odometry_msg->pose.pose.orientation.z).finished()));
//  transforms::Xformd X_X_XAV = X_X_NED_ * X_NED_UAV * X_UAV_XAV_;
//  Vector3d vel_UAV(odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z);
//  Vector3d vel_XAV = q_UAV_XAV_.rotp(vel_UAV);
//  Vector3d omg_UAV(odometry_msg->twist.twist.angular.x, odometry_msg->twist.twist.angular.y, odometry_msg->twist.twist.angular.z);
//  Vector3d omg_XAV = q_UAV_XAV_.rotp(omg_UAV);
//  q_X_UAV_ = X_X_NED_.q() * X_NED_UAV.q();

//  transforms::Quatd q_NED_UAV(Vector4d(odometry_msg->pose.pose.orientation.w,
//                                             odometry_msg->pose.pose.orientation.x,
//                                             odometry_msg->pose.pose.orientation.y,
//                                             odometry_msg->pose.pose.orientation.z));
  transforms::Quatd q_NED_UAV;
  q_NED_UAV.setW(odometry_msg->pose.pose.orientation.w);
  q_NED_UAV.setX(odometry_msg->pose.pose.orientation.x);
  q_NED_UAV.setY(odometry_msg->pose.pose.orientation.y);
  q_NED_UAV.setZ(odometry_msg->pose.pose.orientation.z);

  nav_msgs::Odometry odom_X_XAV;
  odom_X_XAV.header = odometry_msg->header;
  odom_X_XAV.pose.covariance = odometry_msg->pose.covariance; //std::cout << X_X_XAV.t().transpose() << " " << X_X_XAV.q() << " " << vel_XAV.transpose() << " " << omg_XAV.transpose() << std::endl;
  odom_X_XAV.pose.pose.position.x =  odometry_msg->pose.pose.position.x; //X_X_XAV.t().x();
  odom_X_XAV.pose.pose.position.y = -odometry_msg->pose.pose.position.y; //X_X_XAV.t().y();
  odom_X_XAV.pose.pose.position.z = -odometry_msg->pose.pose.position.z; //X_X_XAV.t().z();
  odom_X_XAV.pose.pose.orientation.w =  odometry_msg->pose.pose.orientation.w; //X_X_XAV.q().w();
  odom_X_XAV.pose.pose.orientation.x =  odometry_msg->pose.pose.orientation.x; //X_X_XAV.q().x();
  odom_X_XAV.pose.pose.orientation.y = -odometry_msg->pose.pose.orientation.y; //X_X_XAV.q().y();
  odom_X_XAV.pose.pose.orientation.z = -odometry_msg->pose.pose.orientation.z; //X_X_XAV.q().z();
  odom_X_XAV.twist.covariance = odometry_msg->twist.covariance;
  odom_X_XAV.twist.twist.linear.x =  odometry_msg->twist.twist.linear.x; //vel_XAV.x();
  odom_X_XAV.twist.twist.linear.y = -odometry_msg->twist.twist.linear.y; //vel_XAV.y();
  odom_X_XAV.twist.twist.linear.z = -odometry_msg->twist.twist.linear.z; //vel_XAV.z();
  odom_X_XAV.twist.twist.angular.x =  odometry_msg->twist.twist.angular.x; //omg_XAV.x();
  odom_X_XAV.twist.twist.angular.y = -odometry_msg->twist.twist.angular.y; //omg_XAV.y();
  odom_X_XAV.twist.twist.angular.z = -odometry_msg->twist.twist.angular.z; //omg_XAV.z();

  mav_msgs::EigenOdometry odometry;
  nav_msgs::Odometry odomest_X_XAV, odomest_NED_UAV;
  
  mav_msgs::eigenOdometryFromMsg(odom_X_XAV, &odometry);
  unscented_kalman_filter_.SetOdometryMeasurement(odometry);
  unscented_kalman_filter_.UpdateStatusEstimate();
  
  geometry_msgs::WrenchStamped wrenchmsg_X = unscented_kalman_filter_.getUkfExternalForcesMoments();
//  std::cout << wrenchmsg_X.wrench.torque.x << " " << wrenchmsg_X.wrench.torque.y << " " << wrenchmsg_X.wrench.torque.z << std::endl;
  Vector3d F_NED(wrenchmsg_X.wrench.force.x, -wrenchmsg_X.wrench.force.y, -wrenchmsg_X.wrench.force.z);
  Vector3d F_UAV = q_NED_UAV.rotp(F_NED);
  Vector3d T_NED(wrenchmsg_X.wrench.torque.x, -wrenchmsg_X.wrench.torque.y, -wrenchmsg_X.wrench.torque.z);
//  std::cout << T_NED.transpose() << std::endl;
  Vector3d T_UAV = q_NED_UAV.rotp(T_NED);
//  std::cout << q_NED_UAV << std::endl;
//  std::cout << T_UAV.transpose() << std::endl << std::endl;
  geometry_msgs::WrenchStamped wrenchmsg_UAV;
  wrenchmsg_UAV.header = wrenchmsg_X.header;
  wrenchmsg_UAV.wrench.force.x = F_UAV.x();
  wrenchmsg_UAV.wrench.force.y = F_UAV.y();
  wrenchmsg_UAV.wrench.force.z = F_UAV.z();
  wrenchmsg_UAV.wrench.torque.x = T_UAV.x();
  wrenchmsg_UAV.wrench.torque.y = T_UAV.y();
  wrenchmsg_UAV.wrench.torque.z = -T_UAV.z(); // ...

  external_forces_moments_estimate_pub_.publish(wrenchmsg_UAV);

  if (pub_arrow_)
  {
      double arrow_scale = F_UAV.norm();
      if (arrow_scale > 0.0)
      {
          transforms::Quatd q_arrow = transforms::Quatd::from_two_unit_vectors(Eigen::Vector3d(1.,0.,0.), F_UAV.normalized());
          marker_.scale.x = 0.5 * arrow_scale;
          marker_.pose.orientation.w = q_arrow.w();
          marker_.pose.orientation.x = q_arrow.x();
          marker_.pose.orientation.y = q_arrow.y();
          marker_.pose.orientation.z = q_arrow.z();
          marker_pub_.publish(marker_);
      }
  }
  
  unscented_kalman_filter_.getUkfEstimatedOdometry(&odomest_X_XAV);

  odomest_NED_UAV.header = odomest_X_XAV.header;
  odomest_NED_UAV.pose.covariance = odomest_X_XAV.pose.covariance;
  odomest_NED_UAV.twist.covariance = odomest_X_XAV.twist.covariance;

//  transforms::Xformd Xest_X_XAV((Eigen::Vector3d() << odomest_X_XAV.pose.pose.position.x,
//                                                      odomest_X_XAV.pose.pose.position.y,
//                                                      odomest_X_XAV.pose.pose.position.z).finished(),
//                               transforms::Quatd((Vector4d() << odomest_X_XAV.pose.pose.orientation.w,
//                                                                odomest_X_XAV.pose.pose.orientation.x,
//                                                                odomest_X_XAV.pose.pose.orientation.y,
//                                                                odomest_X_XAV.pose.pose.orientation.z).finished()));
//  transforms::Xformd Xest_NED_UAV = X_X_NED_.inverse() * Xest_X_XAV * X_UAV_XAV_.inverse();
//  Vector3d velest_XAV(odomest_X_XAV.twist.twist.linear.x, odomest_X_XAV.twist.twist.linear.y, odomest_X_XAV.twist.twist.linear.z);
//  Vector3d velest_UAV = q_UAV_XAV_.rota(velest_XAV);
//  Vector3d omgest_XAV(odomest_X_XAV.twist.twist.angular.x, odomest_X_XAV.twist.twist.angular.y, odomest_X_XAV.twist.twist.angular.z);
//  Vector3d omgest_UAV = q_UAV_XAV_.rota(omgest_XAV);

  odomest_NED_UAV.pose.pose.position.x =  odomest_X_XAV.pose.pose.position.x; //Xest_NED_UAV.t().x();
  odomest_NED_UAV.pose.pose.position.y = -odomest_X_XAV.pose.pose.position.y; //Xest_NED_UAV.t().y();
  odomest_NED_UAV.pose.pose.position.z = -odomest_X_XAV.pose.pose.position.z; //Xest_NED_UAV.t().z();
  odomest_NED_UAV.pose.pose.orientation.w =  odomest_X_XAV.pose.pose.orientation.w; //Xest_NED_UAV.q().w();
  odomest_NED_UAV.pose.pose.orientation.x =  odomest_X_XAV.pose.pose.orientation.x; //Xest_NED_UAV.q().x();
  odomest_NED_UAV.pose.pose.orientation.y = -odomest_X_XAV.pose.pose.orientation.y; //Xest_NED_UAV.q().y();
  odomest_NED_UAV.pose.pose.orientation.z = -odomest_X_XAV.pose.pose.orientation.z; //Xest_NED_UAV.q().z();
  odomest_NED_UAV.twist.twist.linear.x =  odomest_X_XAV.twist.twist.linear.x; //velest_UAV.x();
  odomest_NED_UAV.twist.twist.linear.y = -odomest_X_XAV.twist.twist.linear.y; //velest_UAV.y();
  odomest_NED_UAV.twist.twist.linear.z = -odomest_X_XAV.twist.twist.linear.z; //velest_UAV.z();
  odomest_NED_UAV.twist.twist.angular.x =  odomest_X_XAV.twist.twist.angular.x; //omgest_UAV.x();
  odomest_NED_UAV.twist.twist.angular.y = -odomest_X_XAV.twist.twist.angular.y; //omgest_UAV.y();
  odomest_NED_UAV.twist.twist.angular.z = -odomest_X_XAV.twist.twist.angular.z; //omgest_UAV.z();
  
  odometry_estimated_pub_.publish(odomest_NED_UAV);

  external_forces_moment_covariance_pub_.publish(
      unscented_kalman_filter_.getForcesTorqueCovariance());
}

void UnscentedKalmanFilterNode::MotorVelocityMeasureCallback(
    const mav_msgs::Actuators& actuators_msg)
{
  ROS_INFO_STREAM_ONCE(
      std :: setprecision (2) << std :: fixed << "Received first message from acturators\n" << actuators_msg);
  unscented_kalman_filter_.SetMotorSpeedMeasurement(actuators_msg);
}

bool UnscentedKalmanFilterNode::OffsetRemovalServiceCallback(std_srvs::SetBool::Request &req,
                                                             std_srvs::SetBool::Response &resp)
{
  ROS_INFO_STREAM("Unscented kalman filter: received \'ukf_compute_offset\' service call.");
  
  std::string response_msg;
  
  if(req.data == true){
    // compute new offset
    offset_source_ = ForceOffsetSource::COMPUTED;
    response_msg = "Unscented kalman filter: computing new force offset.";
    unscented_kalman_filter_.computeNewForceOffset();

  }else if(req.data == false && offset_source_ != ForceOffsetSource::STORED){
    // use stored offset 
    offset_source_ = ForceOffsetSource::STORED;
    response_msg = "Unscented kalman filter: using force offset from parameters.";
    unscented_kalman_filter_.useStoredForceOffset();
  
  }else{
    // already using stored offset
    offset_source_ = ForceOffsetSource::STORED;
    response_msg = "Unscented kalman filter: already using force offset from parameters.";
    unscented_kalman_filter_.useStoredForceOffset(); // display diagnostic info
  }
  resp.message = response_msg;
  resp.success = true;

  return true;
}

}  //end of namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unscented_kalman_filter_node");

  ros::NodeHandle nh, nh_private("~");
  unscented_kalman_filter::UnscentedKalmanFilterNode unscented_kalman_filter_node_(nh, nh_private);

  ros::spin();

  return 0;

}

