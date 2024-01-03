// Add package headers

// Add ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#ifndef __NAVIGATION_NODE_HEADER__
#define __NAVIGATION_NODE_HEADER__

class navigationNode : public rclcpp::Node
{
public:
    // Contructor
    navigationNode();

    // Provide new attitude estimates to the user
    void getNewEstimate(double &roll, double &pitch, double &yaw);

private:
    // Variables for subscribers
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_Sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_Sub;

    // Subscriber Callback functions
    void baro_SCb(sensor_msgs::msg::FluidPressure msg);
    void imu_SCb(sensor_msgs::msg::Imu msg);

    // Variables to store received data
    double attiW, attiX, attiY, attiZ, angVelX, angVelY, angVelZ, linAccX, linAccY, linAccZ, baroAlt;

    // Variables to store estiated roll, pitch and yaw data
    double roll, pitch, yaw;

    // Complementary filter parameters
    double alf = 0.98;

    // Keep track of last time-stamp
    rclcpp::Time imu_lts, baro_lts;

    // Variables to keep track of sampling time
    rclcpp::Duration imuDt, baroDt;
};

#endif // __NAVIGATION_NODE_HEADER__