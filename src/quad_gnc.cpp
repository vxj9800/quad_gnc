// Add Standard c++ headers
#include <iostream>

// Add ROS headers
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

// Add package headers
#include <quad_gnc/guidance.hpp>
#include <quad_gnc/navigation.hpp>

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

    // Get a shared pointer for animation node object
    std::shared_ptr<navigationNode> navigationNodePtr = std::make_shared<navigationNode>();
    rosExecutor.add_node(navigationNodePtr);

    // Initialize joystick
    Joystick js0;

    // Variables to store joystick states, rt = roll and throttle, yp = yaw and pitch
    joystick_position rt, yp;

    // Variable to keep track of whether the quad is armed or not armed
    bool quadArmed = false;

    while(rclcpp::ok())
    {
        rosExecutor.spin_some();

        // Check if the simulation should be running or not
        quadArmed = js0.buttonPressed(0) ? 0 : quadArmed; // If button 0 was pressed then quad is not armed
        quadArmed = js0.buttonPressed(1) ? 1 : quadArmed; // If button 1 was pressed then quad is armed

        // Get RC input or desired state values
        rt = js0.joystickPosition(0);
        yp = js0.joystickPosition(1);

        // Print the joystick states
        std::cout << "Armed: " << quadArmed << "\tThrottle: " << rt.y << "\tRoll: " << rt.x << "\tPitch: " << yp.y << "\tYaw: " << yp.x << std::endl;

        double roll, pitch, yaw;
        if (navigationNodePtr->getNewData(roll, pitch, yaw)) // If new state estimates are available and 
        { // Send the control signal
            // std::cout << "R: " << roll << "P: " << pitch << "Y: " << yaw << std::endl;
        }
    }

    // Cleanup
    rclcpp::shutdown();
    return 0;
}