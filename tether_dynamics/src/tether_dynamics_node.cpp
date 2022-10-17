#include "tether_dynamics/tether_dynamics.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tether_dynamics_node");
    tether_dynamics::TetherDynamics TD;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
