// Add Standard c++ headers
#include <iostream>

// Add ROS headers
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

// Add package headers
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

    while(rclcpp::ok())
    {
        rosExecutor.spin_some();

        // Get RC input or desired state values
        // Guidance part of the code will come here

        double roll, pitch, yaw;
        if (navigationNodePtr->getNewData(roll, pitch, yaw)) // If new state estimates are available
        { // Send the control signal
            std::cout << "R: " << roll << "P: " << pitch << "Y: " << yaw << std::endl;
        }
    }

    // Cleanup
    rclcpp::shutdown();
    return 0;
}