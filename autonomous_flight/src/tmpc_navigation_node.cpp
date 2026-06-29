/*
    FILE: tmpc_navigation_node.cpp
    ------------------------------------
    ROS node entry for the T-MPC++ navigation stack (benchmark method M6_tmpc).

    AsyncSpinner(3) mirrors im2_mppi_navigation_node so the planning / execution /
    visualization callbacks do not block each other.

    Usage:
        roslaunch autonomous_flight tmpc_demo.launch
*/

#include <ros/ros.h>
#include <autonomous_flight/tmpcNavigation.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tmpc_navigation_node");
    ros::NodeHandle nh;

    AutoFlight::tmpcNavigation navigator(nh);
    navigator.run();   // takeoff() + registerCallback()

    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
