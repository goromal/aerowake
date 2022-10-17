#include "boat_dynamics/boat_dynamics.h"

namespace boat_dynamics {

BoatDynamics::BoatDynamics() : t_prev_(0.0), t_initialized_(false), nh_private_("~")
{
    nh_ = ros::NodeHandle();

    boat_speed_mps_ = nh_private_.param<double>("boat_speed", 0.0);

    if (nh_private_.hasParam("NED2boat_T0_euler"))
    {
        std::vector<double> T0_vec;
        nh_private_.getParam("NED2boat_T0", T0_vec);
        T_NED_boat_ = Xformd((Vector3d() << T0_vec[0], T0_vec[1], T0_vec[2]).finished(),
                             Quatd::from_euler(T0_vec[3], T0_vec[4], T0_vec[5]));
        boat_height_m_ = -T0_vec[2];
    }
    else if (nh_private_.hasParam("NED2boat_T0"))
    {
        std::vector<double> T0_vec;
        nh_private_.getParam("NED2boat_T0", T0_vec);
        T_NED_boat_ = Xformd((Vector3d() << T0_vec[0], T0_vec[1], T0_vec[2]).finished(),
                             Quatd((Vector4d() << T0_vec[3], T0_vec[4], T0_vec[5], T0_vec[6]).finished()));
        boat_height_m_ = -T0_vec[2];
    }
    else
    {
        T_NED_boat_ = Xformd((Vector3d() << 0.0, 0.0, boat_height_m_).finished(), Quatd::Identity());
        boat_height_m_ = 0.3; // 2.0 NOT USED <<<<<<<<<<<<<<<<<<
    }

    truth_pub_ = nh_.advertise<nav_msgs::Odometry>("boat_truth_NED", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("boat_marker", 1);

    transform_.header.frame_id = "NED";
    transform_.child_frame_id = "boat";

    truth_.header.frame_id = "NED";

    marker_.header.frame_id = "NED";
    marker_.id = 1;
    marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.mesh_resource = "package://boat_dynamics/mesh/ship_ned_GCS.dae";
    double boat_scale = 20.0;
    marker_.scale.x = boat_scale;
    marker_.scale.y = boat_scale;
    marker_.scale.z = boat_scale;
    marker_.color.r = 1.0;
    marker_.color.g = 1.0;
    marker_.color.b = 1.0;
    marker_.color.a = 1.0;

    ros::Time Rt = ros::Time::now();
    setMessageStates(Rt);

    timer_ = nh_.createTimer(ros::Duration(ros::Rate(20)), &BoatDynamics::onUpdate, this);
}

void BoatDynamics::onUpdate(const ros::TimerEvent &)
{
    ros::Time Rt = ros::Time::now();
    double t = Rt.toSec();

    if (!t_initialized_)
    {
        t_prev_ = t;
        t_initialized_ = true;
        return;
    }

    double dt = t - t_prev_;
    t_prev_ = t;

    // update boat state
    T_NED_boat_.t_(0) += boat_speed_mps_ * dt;
    // TODO more interesting motion

    // update and send messages
    setMessageStates(Rt);
    tbr_.sendTransform(transform_);
    truth_pub_.publish(truth_);
    marker_pub_.publish(marker_);
}

void BoatDynamics::setMessageStates(ros::Time &rt)
{
    transform_.header.stamp = rt;
    transform_.transform.translation.x = T_NED_boat_.t_(0);
    transform_.transform.translation.y = T_NED_boat_.t_(1);
    transform_.transform.translation.z = T_NED_boat_.t_(2);
    transform_.transform.rotation.w = T_NED_boat_.q_.w();
    transform_.transform.rotation.x = T_NED_boat_.q_.x();
    transform_.transform.rotation.y = T_NED_boat_.q_.y();
    transform_.transform.rotation.z = T_NED_boat_.q_.z();

    truth_.header.stamp = rt;
    truth_.pose.pose.position.x = T_NED_boat_.t_(0);
    truth_.pose.pose.position.y = T_NED_boat_.t_(1);
    truth_.pose.pose.position.z = T_NED_boat_.t_(2);
    truth_.pose.pose.orientation.w = T_NED_boat_.q_.w();
    truth_.pose.pose.orientation.x = T_NED_boat_.q_.x();
    truth_.pose.pose.orientation.y = T_NED_boat_.q_.y();
    truth_.pose.pose.orientation.z = T_NED_boat_.q_.z();
    truth_.twist.twist.linear.x = boat_speed_mps_;
    truth_.twist.twist.linear.y = 0.0;
    truth_.twist.twist.linear.z = 0.0;
    truth_.twist.twist.angular.x = 0.0;
    truth_.twist.twist.angular.y = 0.0;
    truth_.twist.twist.angular.z = 0.0;

    marker_.header.stamp = rt;
    marker_.pose = truth_.pose.pose;
}

} // end namespace boat_dynamics
