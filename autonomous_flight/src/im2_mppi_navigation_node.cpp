/*
    FILE: im2_mppi_navigation_node.cpp
    ------------------------------------
    ROS node entry point for the IM2-MPPI navigation stack.
    Pattern mirrors mpc_navigation_node.cpp exactly.

    Usage:
        roslaunch autonomous_flight im2_mppi_demo.launch
*/

#include <ros/ros.h>
#include <autonomous_flight/im2MppiNavigation.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "im2_mppi_navigation_node");
    ros::NodeHandle nh;

    AutoFlight::im2MppiNavigation navigator(nh);
    navigator.run();   // takeoff() + registerCallback()

    ros::spin();
    return 0;
}
