#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry-utils-lib/xform.h>
#include <geometry-utils-lib/numeric.h>

static geometry_msgs::PoseStamped boat_mocap;
static geometry_msgs::TwistStamped boat_mocap_vel;
static transforms::XformdDifferentiator boat_mocap_diff;
//static transforms::Xformd X_NWU_NED;
//static transforms::Xformd X_BOAT_XOAT;

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

//    transforms::Xformd X_NED_BOAT;
//    poseToXform(msg->pose.pose, X_NED_BOAT);
//    transforms::Xformd X_NWU_XOAT = X_NWU_NED * X_NED_BOAT * X_BOAT_XOAT;
//    boat_mocap.header.stamp = ros::Time::now();
//    boat_mocap.pose.position.x = X_NWU_XOAT.t().x();
//    boat_mocap.pose.position.y = X_NWU_XOAT.t().y();
//    boat_mocap.pose.position.z = X_NWU_XOAT.t().z();
//    boat_mocap.pose.orientation.w = X_NWU_XOAT.q().w();
//    boat_mocap.pose.orientation.x = X_NWU_XOAT.q().x();
//    boat_mocap.pose.orientation.y = X_NWU_XOAT.q().y();
//    boat_mocap.pose.orientation.z = X_NWU_XOAT.q().z();

    // Simplify with known NED-NED <-> NWU-NWU transform conversion
    boat_mocap.header.stamp = ros::Time::now();
    boat_mocap.pose.position.x =  msg->pose.pose.position.x;
    boat_mocap.pose.position.y = -msg->pose.pose.position.y;
    boat_mocap.pose.position.z = -msg->pose.pose.position.z;
    boat_mocap.pose.orientation.w =  msg->pose.pose.orientation.w;
    boat_mocap.pose.orientation.x =  msg->pose.pose.orientation.x;
    boat_mocap.pose.orientation.y = -msg->pose.pose.orientation.y;
    boat_mocap.pose.orientation.z = -msg->pose.pose.orientation.z;
    transforms::Xformd X_NWU_XOAT;
    poseToXform(boat_mocap.pose, X_NWU_XOAT);

    double t = ros::Time::now().toSec();
    double dt = t - t_prev;
    t_prev = t;

    // mocap package appears to do something closer to world-frame (as opposed to body-frame) simple
    // differentiation (but not for quaternion, I believe...)
    Eigen::Matrix<double, 6, 1> boat_twist = boat_mocap_diff.calculate(X_NWU_XOAT, dt);

    boat_mocap_vel.header.stamp = ros::Time::now();
    boat_mocap_vel.twist.linear.x = boat_twist(0, 0);
    boat_mocap_vel.twist.linear.y = boat_twist(1, 0);
    boat_mocap_vel.twist.linear.z = boat_twist(2, 0);
    boat_mocap_vel.twist.angular.x = boat_twist(3, 0);
    boat_mocap_vel.twist.angular.y = boat_twist(4, 0);
    boat_mocap_vel.twist.angular.z = boat_twist(5, 0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulated_mocap_boat");
    ros::NodeHandle nh;

//    X_NWU_NED.sett((Vector3d() << 0., 0., 0.).finished());
//    transforms::Quatd q = transforms::Quatd::from_euler(3.14159265, 0., 0.);
//    X_NWU_NED.setq(q);
//    X_BOAT_XOAT.sett((Vector3d() << 0., 0., 0.).finished());
//    X_BOAT_XOAT.setq(q);

    ros::Subscriber pose_sub = nh.subscribe("boat_truth", 1, odomCallback);
    ros::Publisher  mcap_pub = nh.advertise<geometry_msgs::PoseStamped>("boat_mocap", 1);
    ros::Publisher  mcpv_pub = nh.advertise<geometry_msgs::TwistStamped>("boat_mocap_vel", 1);
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        mcap_pub.publish(boat_mocap);
        mcpv_pub.publish(boat_mocap_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
