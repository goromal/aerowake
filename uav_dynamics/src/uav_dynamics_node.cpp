#include "uav_dynamics/uav_dynamics_ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_dynamics_node");
    uav_dynamics::QuadrotorDynamicsROS QDR;
    ros::spin();
    return 0;
}
