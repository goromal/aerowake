#include <ros/ros.h>
#include <Eigen/Core>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry-utils-lib/quat.h"
#include "math-utils-lib/constants.h"

using namespace transforms;
using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "world_to_NED_broadcaster");
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // frame rotation from world to NED
    Quatd q_0_NED = Quatd::from_axis_angle((Vector3d() << 1.0, 0.0, 0.0).finished(), UTILS_PI);

    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = "NED";
    static_transformStamped.transform.translation.x = 0.0;
    static_transformStamped.transform.translation.y = 0.0;
    static_transformStamped.transform.translation.z = 1.8;
    static_transformStamped.transform.rotation.w = q_0_NED.w();
    static_transformStamped.transform.rotation.x = q_0_NED.x();
    static_transformStamped.transform.rotation.y = q_0_NED.y();
    static_transformStamped.transform.rotation.z = q_0_NED.z();

    static_broadcaster.sendTransform(static_transformStamped);

    ros::spin();

    return 0;
}
