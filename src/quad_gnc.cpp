// Add Standard c++ headers
#include <iostream>

// Add ROS headers
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

// Add package headers
#include <quad_gnc/guidance.hpp>
#include <quad_gnc/navigation.hpp>
#include <quad_gnc/control.hpp>

// If the package name is not defined at compile time then set it to empty
#ifndef ROS_PACKAGE_NAME
#define ROS_PACKAGE_NAME ""
#endif

int main(int argc, char *argv[])
{
    // Some initialization.
    rclcpp::init(argc, argv);

    // Get location of the package share directory
    std::string pkgShareDir = ament_index_cpp::get_package_share_directory(ROS_PACKAGE_NAME);

    // Initialize the ROS executor
    rclcpp::executors::MultiThreadedExecutor rosExecutor;

    // Get a shared pointer for navigation node object
    std::shared_ptr<navigationNode> navigationNodePtr = std::make_shared<navigationNode>();
    rosExecutor.add_node(navigationNodePtr);

    // Get a shared pointer for control node object
    std::shared_ptr<controlNode> controlNodePtr = std::make_shared<controlNode>();
    rosExecutor.add_node(controlNodePtr);

    // Initialize joystick
    Joystick js0;

    // Variables to store joystick states, rt = roll and throttle, yp = yaw and pitch
    joystick_position rt, yp;

    // Variable to keep track of whether the quad is armed or not armed
    bool quadArmed = false;

    while(rclcpp::ok())
    {
        // Get the state estimates from navigation node
        std::vector<double> currAtt(4);
        navigationNodePtr->getNewEstimate(currAtt[1], currAtt[2], currAtt[3]);

        // Get quadcopter arming state
        quadArmed = js0.getArmState();

        // Get desired states
        std::vector<double> desAtt(4);
        js0.getTRPY(desAtt[0], desAtt[1], desAtt[2], desAtt[3]);

        // Compute the control signal and publish it
        controlNodePtr->motVolts_PFn(navigationNodePtr->getTimeStamp(), currAtt, desAtt);
        controlNodePtr->armState_PFn(navigationNodePtr->getTimeStamp(), quadArmed);
    }

    // Cleanup
    rclcpp::shutdown();
    return 0;
}