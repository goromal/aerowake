#include "boat_dynamics/boat_dynamics.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "boat_dynamics_node");
    boat_dynamics::BoatDynamics BD;
    ros::spin();
    return 0;
}
