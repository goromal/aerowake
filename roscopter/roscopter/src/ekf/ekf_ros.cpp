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

#include <ros/package.h>

#include "ekf/state.h"
#include "ekf/ekf_ros.h"
#include "roscopter_utils/yaml.h"
#include "roscopter_utils/gnss.h"

using namespace Eigen;

namespace roscopter
{
namespace ekf
{

EKF_ROS::EKF_ROS() :
    nh_(), nh_private_("~")
{}

EKF_ROS::~EKF_ROS()
{}

void EKF_ROS::initROS()
{
    // Global parameters
    std::string roscopter_path = ros::package::getPath("roscopter");
    std::string parameter_filename = nh_private_.param<std::string>("param_filename", roscopter_path + "/params/ekf.yaml");
    std::string frame_filename = nh_private_.param<std::string>("frame_paramfile", "/no/frame/paramfile/given.yaml");
    bool enable_logging = nh_private_.param<bool>("enable_logging", false);
    std::string log_prefix = nh_private_.param<std::string>("log_prefix", "/no/log/prefix/given");

    init(parameter_filename, frame_filename, enable_logging, log_prefix);

    // Publishers
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    euler_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("euler_degrees", 1);
    imu_bias_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_bias", 1);
    is_flying_pub_ = nh_.advertise<std_msgs::Bool>("is_flying", 1);
    vel_res_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("vel/res", 1);
#ifdef RELATIVE
    NED2REL_pub_  = nh_.advertise<geometry_msgs::QuaternionStamped>("q_NED_REL", 1);
    att_corr_pub_ = nh_.advertise<geometry_msgs::QuaternionStamped>("att_corr", 1);
    alt_corr_pub_ = nh_.advertise<geometry_msgs::PointStamped>("alt_corr", 1);
    base_vel_pub_ = nh_.advertise<geometry_msgs::PointStamped>("est_base_vel", 1);
    est_rel_pose_pub_    = nh_.advertise<geometry_msgs::PoseStamped>("est_rel_pose", 1);
    vis_res_pub_         = nh_.advertise<std_msgs::Float64MultiArray>("rel_pose/res", 1);
    pos_res_pub_         = nh_.advertise<std_msgs::Float64MultiArray>("rel_pos/res", 1);
    svl_res_pub_         = nh_.advertise<std_msgs::Float64MultiArray>("rel_base_vel/res", 1);
    rel_heading_res_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("rel_base_mag/res", 1);
#else
    baro_res_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("baro/res", 1);
    gnss_res_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("gnss/res", 1);
    att_corr_res_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("att_corr/res", 1);
    alt_corr_res_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("alt_corr/res", 1);
#endif

    // Subscribers
    imu_sub_ = nh_.subscribe("imu", 100, &EKF_ROS::imuCallback, this);
    vel_sub_  = nh_.subscribe("vel", 10, &EKF_ROS::velNEDCallback, this);
#ifdef RELATIVE
    vis_sub_  = nh_.subscribe("rel_pose", 10, &EKF_ROS::relativePoseCallback, this);
    pos_sub_  = nh_.subscribe("rel_pos", 10, &EKF_ROS::relativePositionNEDCallback, this);
    svl_sub_  = nh_.subscribe("rel_base_vel", 10, &EKF_ROS::relativeBaseVelNEDCallback, this);
    rel_heading_sub_ = nh_.subscribe("rel_base_mag", 1, &EKF_ROS::relativeBaseMagCallback, this);
    rel_mocap_base_pose_sub_ = nh_.subscribe("rel_mocap_base_pose", 1, &EKF_ROS::relMocapBasePoseCallback, this);
    rel_mocap_base_twist_sub_ = nh_.subscribe("rel_mocap_base_twist", 1, &EKF_ROS::relMocapBaseTwistCallback, this);
    rel_mocap_rover_sub_ = nh_.subscribe("rel_mocap_rover", 1, &EKF_ROS::relMocapRoverCallback, this);
#else
    baro_sub_ = nh_.subscribe("baro", 100, &EKF_ROS::baroCallback, this);
    gnss_sub_ = nh_.subscribe("gnss", 10, &EKF_ROS::gnssCallback, this);
    att_corr_sub_  = nh_.subscribe("att_corr", 10, &EKF_ROS::attitudeCorrectionCallback, this);
    alt_corr_sub_  = nh_.subscribe("alt_corr", 10, &EKF_ROS::altitudeCorrectionCallback, this);
    abs_mocap_sub_ = nh_.subscribe("abs_mocap", 10, &EKF_ROS::absoluteMocapCallback, this);
#endif

    // End ROS initialization
    ros_initialized_ = true;
}

void EKF_ROS::init(const std::string& param_file, const std::string& frame_file, const bool& enable_logs, const std::string& log_prefix)
{
    ekf_.load(param_file, frame_file, enable_logs, log_prefix);

    // Load Sensor Noise Parameters
    double acc_stdev, gyro_stdev;
    get_yaml_node("accel_noise_stdev", param_file, acc_stdev);
    get_yaml_node("gyro_noise_stdev", param_file, gyro_stdev);
    imu_R_.setZero();
    imu_R_.topLeftCorner<3,3>() = acc_stdev * acc_stdev * I_3x3;
    imu_R_.bottomRightCorner<3,3>() = gyro_stdev * gyro_stdev * I_3x3;
    double mocap_pos_stdev, mocap_att_stdev;
    get_yaml_node("mocap_position_noise_stdev", param_file, mocap_pos_stdev);
    get_yaml_node("mocap_attitude_noise_stdev", param_file, mocap_att_stdev);
    get_yaml_node("use_mocap", param_file, use_mocap_);
    get_yaml_node("use_velocity", param_file, use_velocity_);
#ifdef RELATIVE
    get_yaml_node("use_base_mag", param_file, use_base_mag_);
    get_yaml_node("use_rel_pose", param_file, use_relative_pose_);
    get_yaml_node("use_rel_position", param_file, use_relative_position_);
    get_yaml_node("use_base_velocity", param_file, use_base_velocity_);
    double mocap_vel_stdev;
    get_yaml_node("mocap_velocity_noise_stdev", param_file, mocap_vel_stdev);
    R_rel_mocap_.setZero();
    R_rel_mocap_.block<3,3>(0,0) = mocap_pos_stdev * mocap_pos_stdev * I_3x3;
    R_rel_mocap_.block<3,3>(3,3) = mocap_att_stdev * mocap_att_stdev * I_3x3;
    R_rel_mocap_.block<3,3>(6,6) = mocap_att_stdev * mocap_att_stdev * I_3x3;
    R_rel_mocap_.block<3,3>(9,9) = mocap_vel_stdev * mocap_vel_stdev * I_3x3;
    double vis_pos_stdev, vis_att_stdev;
    get_yaml_node("vision_position_noise_stdev", param_file, vis_pos_stdev);
    get_yaml_node("vision_attitude_noise_stdev", param_file, vis_att_stdev);
    vision_R_ << vis_pos_stdev * vis_pos_stdev * I_3x3, Matrix3d::Zero(),
              Matrix3d::Zero(), vis_att_stdev * vis_att_stdev * I_3x3;
    get_yaml_node("manual_mag_noise", param_file, manual_mag_noise_);
    if (manual_mag_noise_)
    {
        double heading_r;
        get_yaml_node("mag_noise_stdev", param_file, heading_r);
        heading_R_ = heading_r * heading_r * I_3x3;
    }
    X_I2base_ = xform::Xformd::Identity();
    v_base_I_ = Vector3d::Zero();
    q_S_B_ = quat::Quatd::Identity();
    t_vision_received_ = -1.0e4;
    t_mocap_base_pose_received_ = -1.0e4;
    rel_pose_pos_r_ = Vector3d::Ones(3,1) * 1.0e4;
    rel_pose_att_r_ = Vector3d::Ones(3,1) * 1.0e4;
    mocap_r_ = Matrix<double, 12, 1>::Ones(12, 1) * 1.0e4;
#else
    get_yaml_node("use_gnss", param_file, use_gnss_);
    get_yaml_node("use_baro", param_file, use_baro_);
    double baro_pressure_stdev;
    get_yaml_node("baro_pressure_noise_stdev", param_file, baro_pressure_stdev);
    baro_R_ = baro_pressure_stdev * baro_pressure_stdev;
    get_yaml_node("use_att_corr", param_file, use_attitude_correction_);
    get_yaml_node("use_alt_corr", param_file, use_altitude_correction_);
    double vis_att_corr_stdev;
    get_yaml_node("attitude_correction_noise_stdev", param_file, vis_att_corr_stdev);
    att_corr_R_ << vis_att_corr_stdev * vis_att_corr_stdev * I_3x3;
    R_abs_mocap_ << mocap_pos_stdev * mocap_pos_stdev * I_3x3,   Matrix3d::Zero(),
                 Matrix3d::Zero(), mocap_att_stdev * mocap_att_stdev * I_3x3;
    double vis_alt_corr_stdev;
    get_yaml_node("altitude_correction_noise_stdev", param_file, vis_alt_corr_stdev);
    alt_corr_R_ = vis_alt_corr_stdev * vis_alt_corr_stdev;
    mocap_r_ = Vector6d::Ones(6,1) * 1.0e4;
#endif

    // End initialization
    start_time_.fromSec(0.0);
}

void EKF_ROS::publishEstimates(const sensor_msgs::ImuConstPtr &msg)
{
    // Only publish is_flying is true once
    if (!is_flying_)
    {
        is_flying_ = ekf_.isFlying();
        if (is_flying_)
        {
            is_flying_msg_.data = is_flying_;
            is_flying_pub_.publish(is_flying_msg_);
        }
    }

    // Only publish states once they start to make sense
    if (use_mocap_ && (mocap_r_.array() > MOCAP_GATE).any())
        return;

    // Pub Odom
    odom_msg_.header = msg->header;

    const State state_est = ekf_.x();
    const ekf::dxMat cov = ekf_.P();

    odom_msg_.pose.pose.position.x = state_est.p(0);
    odom_msg_.pose.pose.position.y = state_est.p(1);
    odom_msg_.pose.pose.position.z = state_est.p(2);

    odom_msg_.pose.pose.orientation.w = state_est.q.w();
    odom_msg_.pose.pose.orientation.x = state_est.q.x();
    odom_msg_.pose.pose.orientation.y = state_est.q.y();
    odom_msg_.pose.pose.orientation.z = state_est.q.z();

    odom_msg_.twist.twist.linear.x = state_est.v(0);
    odom_msg_.twist.twist.linear.y = state_est.v(1);
    odom_msg_.twist.twist.linear.z = state_est.v(2);

    odom_msg_.twist.twist.angular.x = state_est.w(0);
    odom_msg_.twist.twist.angular.y = state_est.w(1);
    odom_msg_.twist.twist.angular.z = state_est.w(2);

    for (unsigned int i = 0; i < 6; i++)
        odom_msg_.pose.covariance[7*i] = cov(i,i);

    for (unsigned int i = 0; i < 3; i++)
        odom_msg_.twist.covariance[7*i] = cov(6+i,6+i);

    // NO COVARIANCE FOR ANGULAR RATE...

    odometry_pub_.publish(odom_msg_);

    // Pub Euler Attitude
    euler_msg_.header = msg->header;
    const Eigen::Vector3d euler_angles = state_est.q.euler() * 180. / M_PI;
    euler_msg_.vector.x = euler_angles(0);
    euler_msg_.vector.y = euler_angles(1);
    euler_msg_.vector.z = euler_angles(2);

    euler_pub_.publish(euler_msg_);

    // Pub Imu Bias estimate
    imu_bias_msg_.header = msg->header;

    imu_bias_msg_.angular_velocity.x = state_est.bg(0);
    imu_bias_msg_.angular_velocity.y = state_est.bg(1);
    imu_bias_msg_.angular_velocity.z = state_est.bg(2);

    imu_bias_msg_.linear_acceleration.x = state_est.ba(0);
    imu_bias_msg_.linear_acceleration.y = state_est.ba(1);
    imu_bias_msg_.linear_acceleration.z = state_est.ba(2);

    imu_bias_pub_.publish(imu_bias_msg_);

#ifdef RELATIVE
    // Pub Rotation from NED to Relative Frame estimate
    NED2REL_msg_.header.stamp = ros::Time::now();
    NED2REL_msg_.quaternion.w = state_est.qREL.w();
    NED2REL_msg_.quaternion.x = state_est.qREL.x();
    NED2REL_msg_.quaternion.y = state_est.qREL.y();
    NED2REL_msg_.quaternion.z = state_est.qREL.z();
    NED2REL_pub_.publish(NED2REL_msg_);

    // Base NED velocity estimate
    base_vel_msg_.header.stamp = ros::Time::now();
    base_vel_msg_.header.frame_id = "NED";
    base_vel_msg_.point.x = state_est.sv.x();
    base_vel_msg_.point.y = state_est.sv.y();
    base_vel_msg_.point.z = state_est.sv.z();
    base_vel_pub_.publish(base_vel_msg_);

    // Relative Pose Estimate
    q_S_B_ = state_est.qREL.inverse() * state_est.q;
    est_rel_pose_msg_.header.stamp = ros::Time::now();
    est_rel_pose_msg_.pose.position.x = state_est.p.x();
    est_rel_pose_msg_.pose.position.y = state_est.p.y();
    est_rel_pose_msg_.pose.position.z = state_est.p.z();
    est_rel_pose_msg_.pose.orientation.w = q_S_B_.w();
    est_rel_pose_msg_.pose.orientation.x = q_S_B_.x();
    est_rel_pose_msg_.pose.orientation.y = q_S_B_.y();
    est_rel_pose_msg_.pose.orientation.z = q_S_B_.z();
    est_rel_pose_pub_.publish(est_rel_pose_msg_);

    // LOGIC FOR SENDING A(L/T)TITUDE CORRECTIONS
    if (start_time_.sec != 0)
    {
        double t = (ros::Time::now() - start_time_).toSec();
        if (abs(t - t_vision_received_) < VISION_CORRECTION_AGE_SECS)
        {
            if (!(rel_pose_att_r_.array() > VIS_ATT_GATE).any())
            {
                geometry_msgs::QuaternionStamped att_corr_msg;
                att_corr_msg.header.stamp = ros::Time::now();
                att_corr_msg.quaternion.w = state_est.q.w();
                att_corr_msg.quaternion.x = state_est.q.x();
                att_corr_msg.quaternion.y = state_est.q.y();
                att_corr_msg.quaternion.z = state_est.q.z();
                att_corr_pub_.publish(att_corr_msg);
            }

            if (!(rel_pose_pos_r_.array() > VIS_POS_GATE).any())
            {
                geometry_msgs::PointStamped alt_corr_msg;
                alt_corr_msg.header.stamp = ros::Time::now();
                alt_corr_msg.header.frame_id = "NED";
                alt_corr_msg.point.x = 0.0;
                alt_corr_msg.point.y = 0.0;
                alt_corr_msg.point.z = state_est.p.z();
                alt_corr_pub_.publish(alt_corr_msg);
            }
        }
    }
#endif
}

void EKF_ROS::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    if (start_time_.sec == 0)
    {
        start_time_ = ros::Time::now();
    }

    Vector6d z;
    z << msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z,
    msg->angular_velocity.x,
    msg->angular_velocity.y,
    msg->angular_velocity.z;

    if ((z.array() != z.array()).any())
    {
        // NaNs in the IMU message
        return;
    }

    double t = (ros::Time::now() - start_time_).toSec();
    ekf_.imuCallback(t, z, imu_R_);

    if(ros_initialized_)
        publishEstimates(msg);
}

void EKF_ROS::velNEDCallback(const ublox_msgs::NavVELNEDConstPtr &msg)
{
    if (start_time_.sec == 0 || !use_velocity_)
        return;

    Vector3d z;
    z << 1.0e-2 * msg->velN, 1.0e-2 * msg->velE, 1.0e-2 * msg->velD;

    const double t = (ros::Time::now() - start_time_).toSec();

    double vel_stdev = 1.0e-2 * msg->sAcc;
    vel_R_ = vel_stdev * vel_stdev * I_3x3;

    Vector3d r = ekf_.velocityNEDUpdate(t, z, vel_R_);
    std_msgs::Float64MultiArray r_msg;
    for (int i = 0; i < 3; i++)
        r_msg.data.push_back(r(i));
    vel_res_pub_.publish(r_msg);
}

#ifdef RELATIVE

void EKF_ROS::relativePoseCallback(const vision::VisionPoseConstPtr &msg)
{
    if (start_time_.sec == 0 || !use_relative_pose_)
        return;

    bool solved = (msg->sol_status != vision::VisionPose::NO_SOLUTION);
    bool valid = msg->dynamically_valid;
    bool outlier = msg->outlier;

    if (solved && valid && !outlier)
    {
        const double t = (ros::Time::now() - start_time_).toSec();
        t_vision_received_ = t;
        Vector3d trans(msg->transform.translation.x,
                       msg->transform.translation.y,
                       msg->transform.translation.z);
        Vector4d quatc(msg->transform.rotation.w,
                       msg->transform.rotation.x,
                       msg->transform.rotation.y,
                       msg->transform.rotation.z);
        xform::Xformd z(trans, quat::Quatd(quatc));
        Vector6d r = ekf_.relativePoseUpdate(t, z, vision_R_);
        rel_pose_pos_r_ = r.segment<3>(0);
        rel_pose_att_r_ = r.segment<3>(3);
        std_msgs::Float64MultiArray r_msg;
        for (int i = 0; i < 6; i++)
            r_msg.data.push_back(r(i));
        vis_res_pub_.publish(r_msg);
    }
}

void EKF_ROS::relativePositionNEDCallback(const ublox_msgs::NavRELPOSNEDConstPtr &msg)
{
    if (start_time_.sec == 0 || !use_relative_position_)
        return;

    double N = 1.0e-2 * msg->relPosN + 1.0e-4 * msg->relPosHPN;
    double E = 1.0e-2 * msg->relPosE + 1.0e-4 * msg->relPosHPE;
    double D = 1.0e-2 * msg->relPosD + 1.0e-4 * msg->relPosHPD;

    Vector3d z;
    z << N, E, D;

    double x_pos_stdev = 1.0e-4 * msg->accN;
    double y_pos_stdev = 1.0e-4 * msg->accE;
    double z_pos_stdev = 1.0e-4 * msg->accD;

    pos_R_ = Eigen::DiagonalMatrix<double, 3>(x_pos_stdev*x_pos_stdev,
             y_pos_stdev*y_pos_stdev,
             z_pos_stdev*z_pos_stdev);

    const double t = (ros::Time::now() - start_time_).toSec();

    Vector3d r = ekf_.relativePositionNEDUpdate(t, z, pos_R_);

    std_msgs::Float64MultiArray r_msg;
    for (int i = 0; i < 3; i++)
        r_msg.data.push_back(r(i));
    pos_res_pub_.publish(r_msg);
}

void EKF_ROS::relativeBaseVelNEDCallback(const ublox_msgs::NavVELNEDConstPtr &msg)
{
    if (start_time_.sec == 0 || !use_base_velocity_)
        return;

    Vector3d z;
    z << 1.0e-2 * msg->velN, 1.0e-2 * msg->velE, 1.0e-2 * msg->velD;

    const double t = (ros::Time::now() - start_time_).toSec();

    double vel_stdev = 1.0e-2 * msg->sAcc;
    vel_R_ = vel_stdev * vel_stdev * I_3x3;

    Vector3d r = ekf_.relativeBaseVelNEDUpdate(t, z, vel_R_);

    std_msgs::Float64MultiArray r_msg;
    for (int i = 0; i < 3; i++)
        r_msg.data.push_back(r(i));
    svl_res_pub_.publish(r_msg);
}

void EKF_ROS::relativeBaseMagCallback(const sensor_msgs::MagneticFieldConstPtr &msg)
{
    if (start_time_.sec == 0 || !use_base_mag_)
        return;
    Eigen::Vector3d z(msg->magnetic_field.x,
                      msg->magnetic_field.y,
                      msg->magnetic_field.z);
    const double t = (ros::Time::now() - start_time_).toSec();
    if (!manual_mag_noise_)
    {
        heading_R_ << msg->magnetic_field_covariance[0],
                   msg->magnetic_field_covariance[1],
                   msg->magnetic_field_covariance[2],
                   msg->magnetic_field_covariance[3],
                   msg->magnetic_field_covariance[4],
                   msg->magnetic_field_covariance[5],
                   msg->magnetic_field_covariance[6],
                   msg->magnetic_field_covariance[7],
                   msg->magnetic_field_covariance[8];
    }

    Eigen::Vector3d r = ekf_.relativeBaseMagUpdate(t, z, heading_R_);
    std_msgs::Float64MultiArray r_msg;
    for (int i = 0; i < 3; i++)
        r_msg.data.push_back(r(i));
    rel_heading_res_pub_.publish(r_msg);
}

void EKF_ROS::relMocapBasePoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if (!use_mocap_)
        return;

    X_I2base_.arr_ << msg->pose.position.x,
                   msg->pose.position.y,
                   msg->pose.position.z,
                   msg->pose.orientation.w,
                   msg->pose.orientation.x,
                   msg->pose.orientation.y,
                   msg->pose.orientation.z;

    t_mocap_base_pose_received_ = (ros::Time::now() - start_time_).toSec();
}

void EKF_ROS::relMocapBaseTwistCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    if (!use_mocap_)
        return;

    v_base_I_ << msg->twist.linear.x,
              msg->twist.linear.y,
              msg->twist.linear.z;
}

void EKF_ROS::relMocapRoverCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    const double t = (ros::Time::now() - start_time_).toSec();
    if (start_time_.sec == 0 || !use_mocap_ || (abs(t - t_mocap_base_pose_received_) > REL_MOCAP_AGE_SECS))
        return;

    xform::Xformd X_I_B;
    X_I_B.arr_ << msg->pose.position.x,
               msg->pose.position.y,
               msg->pose.position.z,
               msg->pose.orientation.w,
               msg->pose.orientation.x,
               msg->pose.orientation.y,
               msg->pose.orientation.z;


    const Vector3d z_rel_pos_I = X_I_B.t() - X_I2base_.t();
    const quat::Quatd z_rib = X_I_B.q();
    const quat::Quatd z_ris = X_I2base_.q();
    const Vector3d z_base_vel = v_base_I_;

    mocap_r_ = ekf_.relativeMocapUpdate(t, z_rel_pos_I, z_rib, z_ris, z_base_vel, R_rel_mocap_);
}

#else

void EKF_ROS::baroCallback(const rosflight_msgs::BarometerConstPtr& msg)
{
    if (start_time_.sec == 0 || !use_baro_)
        return;

    const double pressure_meas = static_cast<double>(msg->pressure);
    const double temperature_meas = static_cast<double>(msg->temperature) + 273.0; // ASSUMED CELSIUS, CONVERTING TO KELVIN <<<<
    const double alt = static_cast<double>(msg->altitude);
    const static double alt_comp = 0.25;
    static bool incorporate_baro = false;

    if (!ekf_.groundTempPressSet()/* && abs(alt) < alt_comp*/) // !!!! ONLY BECAUSE RAW BARO DATA NOT CALIBRATED!!!! CHANGE ME!!!!
    {
        std::cout << "Set ground pressure and temp" << std::endl;
        std::cout << "press: " << pressure_meas << std::endl;
        ekf_.setGroundTempPressure(temperature_meas, pressure_meas);
        incorporate_baro = true;
    }

    if (incorporate_baro)
    {
        const double t = (ros::Time::now() - start_time_).toSec();
        double r = ekf_.baroUpdate(t, pressure_meas, baro_R_, temperature_meas);
        std_msgs::Float64MultiArray r_msg;
        r_msg.data.push_back(r);
        baro_res_pub_.publish(r_msg);
    }
    else
    {
        ROS_WARN_THROTTLE(8, "[EKF_ROS] Barometer messages received, but not incorporated because a calibration resulting in alt < 0.25 has not been performed.");
    }
}

void EKF_ROS::gnssCallback(const ublox_msgs::NavPVTConstPtr &msg)
{
    if (start_time_.sec == 0 || !use_gnss_)
        return;

    Vector6d z;

    // lat, lon, alt in (rad, m)
    double lat = M_PI * 1.0e-7 * msg->lat / 180.0;
    double lon = M_PI * 1.0e-7 * msg->lon / 180.0;
    double alt = 1.0e-3 * msg->height;
    Vector3d lla(lat, lon, alt);

    // velocity in NED frame (m/s)
    double v_N = 1.0e-3 * msg->velN;
    double v_E = 1.0e-3 * msg->velE;
    double v_D = 1.0e-3 * msg->velD;
    Vector3d nedvel(v_N, v_E, v_D);

    // Set reference
    if (!ekf_.refLlaSet())
    {
        // set ref lla to first gps position
        ekf_.setRefLla(lla);
        X_e2n_ = x_ecef2ned(lla2ecef(lla));
    }

    // Convert measurements to Earth-centered, Earth-fixed frame (ECEF)
    Vector3d ecefpos = lla2ecef(lla);
    Vector3d ecefvel = X_e2n_.rota(nedvel);

    // Construct measurement
    z << ecefpos.x(),
    ecefpos.y(),
    ecefpos.z(),
    ecefvel.x(),
    ecefvel.y(),
    ecefvel.z();

    // rotate covariance into the ECEF frame
    Vector6d Sigma_diag_NED;
    double horizontal_accuracy = 1.0e-3 * msg->hAcc;
    double vertical_accuracy   = 1.0e-3 * msg->vAcc;
    double speed_accuracy      = 1.0e-3 * msg->sAcc;
    Sigma_diag_NED << horizontal_accuracy,
                   horizontal_accuracy,
                   vertical_accuracy,
                   speed_accuracy,
                   speed_accuracy,
                   speed_accuracy;

    Sigma_diag_NED = Sigma_diag_NED.cwiseProduct(Sigma_diag_NED);
    Matrix3d R_e2n = q_e2n(ecef2lla(z.head<3>())).R();

    Matrix6d Sigma_ecef;
    Sigma_ecef << R_e2n.transpose() * Sigma_diag_NED.head<3>().asDiagonal() * R_e2n, Matrix3d::Zero(),
               Matrix3d::Zero(), R_e2n.transpose() *  Sigma_diag_NED.tail<3>().asDiagonal() * R_e2n;

    double t = (ros::Time::now() - start_time_).toSec();
    Matrix<double, 6, 1> r = ekf_.gnssUpdate(t, z, Sigma_ecef);
    std_msgs::Float64MultiArray r_msg;
    for (int i = 0; i < 6; i++)
        r_msg.data.push_back(r(i));
    gnss_res_pub_.publish(r_msg);
}

void EKF_ROS::attitudeCorrectionCallback(const geometry_msgs::QuaternionStampedConstPtr &msg)
{
    if (start_time_.sec == 0 || !use_attitude_correction_)
        return;

    const double t = (ros::Time::now() - start_time_).toSec();
    quat::Quatd z((Vector4d() << msg->quaternion.w,
                   msg->quaternion.x,
                   msg->quaternion.y,
                   msg->quaternion.z).finished());
    Vector3d r = ekf_.attitudeCorrectionUpdate(t, z, att_corr_R_);
    std_msgs::Float64MultiArray r_msg;
    for (int i = 0; i < 3; i++)
        r_msg.data.push_back(r(i));
    att_corr_res_pub_.publish(r_msg);
}

void EKF_ROS::altitudeCorrectionCallback(const geometry_msgs::PointStampedConstPtr &msg)
{
    if (start_time_.sec == 0 || !use_altitude_correction_)
        return;

    const double t = (ros::Time::now() - start_time_).toSec();
    double z = msg->point.z; // inertial DOWN, assumed to coincide with ship-relative DOWN
    double r = ekf_.altitudeCorrectionUpdate(t, z, alt_corr_R_);
    std_msgs::Float64MultiArray r_msg;
    r_msg.data.push_back(r);
    alt_corr_res_pub_.publish(r_msg);
}

void EKF_ROS::absoluteMocapCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if (start_time_.sec == 0 || !use_mocap_)
        return;

    xform::Xformd z;
    z.arr_ << msg->pose.position.x,
           msg->pose.position.y,
           msg->pose.position.z,
           msg->pose.orientation.w,
           msg->pose.orientation.x,
           msg->pose.orientation.y,
           msg->pose.orientation.z;

    double t = (msg->header.stamp - start_time_).toSec();
    mocap_r_ = ekf_.absoluteMocapUpdate(t, z, R_abs_mocap_);
}
#endif

void EKF_ROS::statusCallback(const rosflight_msgs::StatusConstPtr &msg)
{
    if (msg->armed)
    {
        ekf_.setArmed();
    }
    else
    {
        ekf_.setDisarmed();
    }
}

}

}
