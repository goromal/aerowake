#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry-utils-lib/xform.h>
#include <geometry-utils-lib/numeric.h>

static geometry_msgs::PoseStamped uav_mocap;
static geometry_msgs::TwistStamped uav_mocap_vel;
static transforms::XformdDifferentiator uav_mocap_diff;
static transforms::Xformd X_NWU_NED;
static transforms::Xformd X_UAV_XAV;

void poseToXform(const geometry_msgs::Pose &pose, transforms::Xformd &xform)
{
    double x = pose.position.x;
    double y = pose.position.y;
    double z = pose.position.z;
    double qw = pose.orientation.w;
    double qx = pose.orientation.x;
    double qy = pose.orientation.y;
    double qz = pose.orientation.z;
    Eigen::Vector3d trans(x, y, z);
    transforms::Quatd quat((Eigen::Vector4d() << qw, qx, qy, qz).finished());
    xform.sett(trans);
    xform.setq(quat);
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    static double t_prev = ros::Time::now().toSec();

    transforms::Xformd X_NED_UAV;
    poseToXform(msg->pose.pose, X_NED_UAV);
    transforms::Xformd X_NWU_XAV = X_NWU_NED * X_NED_UAV * X_UAV_XAV;
    uav_mocap.header.stamp = ros::Time::now();
    uav_mocap.pose.position.x = X_NWU_XAV.t().x();
    uav_mocap.pose.position.y = X_NWU_XAV.t().y();
    uav_mocap.pose.position.z = X_NWU_XAV.t().z();
    uav_mocap.pose.orientation.w = X_NWU_XAV.q().w();
    uav_mocap.pose.orientation.x = X_NWU_XAV.q().x();
    uav_mocap.pose.orientation.y = X_NWU_XAV.q().y();
    uav_mocap.pose.orientation.z = X_NWU_XAV.q().z();

//    Vector3d vel_UAV(msg->twist.twist.linear.x,
//                     msg->twist.twist.linear.y,
//                     msg->twist.twist.linear.z);
//    Vector3d omg_UAV(msg->twist.twist.angular.x,
//                     msg->twist.twist.angular.y,
//                     msg->twist.twist.angular.z);
//    Vector3d vel_XAV = X_UAV_XAV.q().rotp(vel_UAV);
//    Vector3d omg_XAV = X_UAV_XAV.q().rotp(omg_UAV);

//    uav_mocap_vel.header.stamp = ros::Time::now();
//    uav_mocap_vel.twist.linear.x = vel_XAV.x();
//    uav_mocap_vel.twist.linear.y = vel_XAV.y();
//    uav_mocap_vel.twist.linear.z = vel_XAV.z();
//    uav_mocap_vel.twist.angular.x = omg_XAV.x();
//    uav_mocap_vel.twist.angular.y = omg_XAV.y();
//    uav_mocap_vel.twist.angular.z = omg_XAV.z();

    double t = ros::Time::now().toSec();
    double dt = t - t_prev;
    t_prev = t;

    // mocap package appears to do something closer to world-frame (as opposed to body-frame) simple
    // differentiation (but not for quaternion, I believe...)
    Eigen::Matrix<double, 6, 1> uav_twist = uav_mocap_diff.calculate(X_NWU_XAV, dt);

    uav_mocap_vel.header.stamp = ros::Time::now();
    uav_mocap_vel.twist.linear.x = uav_twist(0, 0);
    uav_mocap_vel.twist.linear.y = uav_twist(1, 0);
    uav_mocap_vel.twist.linear.z = uav_twist(2, 0);
    uav_mocap_vel.twist.angular.x = uav_twist(3, 0);
    uav_mocap_vel.twist.angular.y = uav_twist(4, 0);
    uav_mocap_vel.twist.angular.z = uav_twist(5, 0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulated_mocap_uav");
    ros::NodeHandle nh;

    X_NWU_NED.sett((Vector3d() << 0., 0., 0.).finished());
    transforms::Quatd q = transforms::Quatd::from_euler(3.14159265, 0., 0.);
    X_NWU_NED.setq(q);
    X_UAV_XAV = X_NWU_NED;

    ros::Subscriber odom_sub = nh.subscribe("uav_truth", 1, odomCallback);
    ros::Publisher  mcap_pub = nh.advertise<geometry_msgs::PoseStamped>("uav_mocap", 1);
    ros::Publisher  mcpv_pub = nh.advertise<geometry_msgs::TwistStamped>("uav_mocap_vel", 1);
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        mcap_pub.publish(uav_mocap);
        mcpv_pub.publish(uav_mocap_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
