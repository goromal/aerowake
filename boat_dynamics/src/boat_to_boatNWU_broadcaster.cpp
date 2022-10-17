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
    Quatd q_boat_boatNWU = Quatd::from_axis_angle((Vector3d() << 1.0, 0.0, 0.0).finished(), UTILS_PI);

    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "boat";
    static_transformStamped.child_frame_id = "boatNWU";
    static_transformStamped.transform.translation.x = 0.0;
    static_transformStamped.transform.translation.y = 0.0;
    static_transformStamped.transform.translation.z = 0.0;
    static_transformStamped.transform.rotation.w = q_boat_boatNWU.w();
    static_transformStamped.transform.rotation.x = q_boat_boatNWU.x();
    static_transformStamped.transform.rotation.y = q_boat_boatNWU.y();
    static_transformStamped.transform.rotation.z = q_boat_boatNWU.z();

    static_broadcaster.sendTransform(static_transformStamped);

    ros::spin();

    return 0;
}
