// BSD 3-Clause License
//
// Copyright (c) 2017, James Jackson, BYU MAGICC Lab, Provo UT
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#pragma once


#include "ekf.h"

#include <mutex>
#include <deque>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <rosflight_msgs/Barometer.h>
#include <rosflight_msgs/Status.h>
#include <ublox_msgs/NavPVT.h>
#include <ublox_msgs/NavVELNED.h>
#include <ublox_msgs/NavRELPOSNED.h>
#include <vision/VisionPose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/QuaternionStamped.h>

#ifdef RELATIVE
#define VISION_CORRECTION_AGE_SECS 1.50
#define REL_MOCAP_AGE_SECS 0.20
#define VIS_POS_GATE 0.4
#define VIS_ATT_GATE 0.05
#endif

#define MOCAP_GATE 0.075

namespace roscopter
{
namespace ekf
{
class EKF_ROS
{
public:

    EKF_ROS();
    ~EKF_ROS();
    void init(const std::string& param_file, const std::string& frame_file, const bool& enable_logs, const std::string& log_prefix);
    void initROS();

    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void velNEDCallback(const ublox_msgs::NavVELNEDConstPtr &msg);
#ifdef RELATIVE
    void relativePoseCallback(const vision::VisionPoseConstPtr &msg);
    void relativePositionNEDCallback(const ublox_msgs::NavRELPOSNEDConstPtr &msg);
    void relativeBaseVelNEDCallback(const ublox_msgs::NavVELNEDConstPtr &msg);
    void relativeBaseMagCallback(const sensor_msgs::MagneticFieldConstPtr& msg);
    void relMocapBasePoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void relMocapBaseTwistCallback(const geometry_msgs::TwistStampedConstPtr &msg);
    void relMocapRoverCallback(const geometry_msgs::PoseStampedConstPtr &msg);
#else
    void baroCallback(const rosflight_msgs::BarometerConstPtr& msg);
    void gnssCallback(const ublox_msgs::NavPVTConstPtr &msg);
    void attitudeCorrectionCallback(const geometry_msgs::QuaternionStampedConstPtr &msg);
    void altitudeCorrectionCallback(const geometry_msgs::PointStampedConstPtr &msg);
    void absoluteMocapCallback(const geometry_msgs::PoseStampedConstPtr &msg);
#endif
    void statusCallback(const rosflight_msgs::StatusConstPtr& msg);

private:
    EKF ekf_;

    ros::Time last_imu_update_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber imu_sub_;
    ros::Subscriber vel_sub_;
    ros::Publisher  vel_res_pub_;
#ifdef RELATIVE
    ros::Subscriber vis_sub_;
    ros::Publisher  vis_res_pub_;
    ros::Publisher  att_corr_pub_;
    ros::Publisher  alt_corr_pub_;
    ros::Publisher  est_rel_pose_pub_;
    ros::Subscriber pos_sub_;
    ros::Publisher  pos_res_pub_;
    ros::Subscriber svl_sub_;
    ros::Publisher  svl_res_pub_;
    ros::Subscriber rel_heading_sub_;
    ros::Publisher  rel_heading_res_pub_;
    ros::Subscriber rel_mocap_base_pose_sub_;
    ros::Subscriber rel_mocap_base_twist_sub_;
    ros::Subscriber rel_mocap_rover_sub_;
#else
    ros::Subscriber baro_sub_;
    ros::Publisher  baro_res_pub_;
    ros::Subscriber gnss_sub_;
    ros::Publisher  gnss_res_pub_;
    ros::Subscriber att_corr_sub_;
    ros::Publisher  att_corr_res_pub_;
    ros::Subscriber alt_corr_sub_;
    ros::Publisher  alt_corr_res_pub_;
    ros::Subscriber abs_mocap_sub_;
#endif

    ros::Subscriber status_sub_;
    ros::Publisher odometry_pub_;
    ros::Publisher euler_pub_;
    ros::Publisher imu_bias_pub_;
    ros::Publisher is_flying_pub_;

#ifdef RELATIVE
    ros::Publisher NED2REL_pub_;
    quat::Quatd q_S_B_;
    ros::Publisher base_vel_pub_;
    geometry_msgs::PointStamped base_vel_msg_;
    geometry_msgs::PoseStamped est_rel_pose_msg_;
    geometry_msgs::QuaternionStamped NED2REL_msg_;
    geometry_msgs::QuaternionStamped att_corr_msg_;
#endif

    sensor_msgs::Imu imu_bias_msg_;
    nav_msgs::Odometry odom_msg_;
    geometry_msgs::Vector3Stamped euler_msg_;
    std_msgs::Bool is_flying_msg_;

    bool imu_init_ = false;
    bool truth_init_ = false;
    bool ros_initialized_ = false;

    bool is_flying_ = false;
    bool armed_ = false;
    ros::Time start_time_;

    Vector6d imu_;
    Matrix6d imu_R_;
    Eigen::Matrix3d vel_R_;
    bool use_velocity_;
#ifdef RELATIVE
    bool use_relative_pose_;
    bool use_relative_position_;
    bool use_base_velocity_;
    bool use_base_mag_;
    double t_vision_received_;
    Matrix6d vision_R_;
    Eigen::Matrix3d pos_R_;
    bool manual_mag_noise_;
    Eigen::Matrix3d heading_R_;
    xform::Xformd X_I2base_;
    Eigen::Vector3d v_base_I_;
    Eigen::Matrix<double, 12, 12> R_rel_mocap_;
    double t_mocap_base_pose_received_;
    Vector3d rel_pose_pos_r_;
    Vector3d rel_pose_att_r_;
    Matrix<double, 12, 1> mocap_r_;
#else
    bool use_baro_;
    bool use_gnss_;
    bool use_attitude_correction_;
    bool use_altitude_correction_;
    xform::Xformd X_e2n_;
    double baro_R_;
    double gnss_horizontal_stdev_;
    double gnss_vertical_stdev_;
    Eigen::Matrix3d att_corr_R_;
    double alt_corr_R_;
    Matrix6d R_abs_mocap_;
    Vector6d mocap_r_;
#endif
    bool use_mocap_;

    void publishEstimates(const sensor_msgs::ImuConstPtr &msg);
};

}
}
