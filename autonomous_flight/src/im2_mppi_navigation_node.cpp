/*
    FILE: im2_mppi_navigation_node.cpp
    ------------------------------------
    ROS node entry for the IM2-MPPI navigation stack.

    Uses ros::AsyncSpinner(3) so that the slow predictor callback (predCB)
    does NOT block the planning/execution/visualization callbacks.

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

    // Three threads:
    //   - one for slow predCB (~200 ms inference)
    //   - one for mppiCB / plan()
    //   - one for trajExeCB (100 Hz) / visCB / odomCB
    //   Concurrent access to the planner is protected by planMutex_.
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
