// Add package headers

// Add ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
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
    int64_t getTimeStamp();

    // Check if data for all subscribers with same time-stamp has been received
    bool isInSync();

private:
    // Variables for subscribers
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr tick_Sub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_Sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_Sub;
    // rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_Sub;
    // rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_Sub;

    // Subscriber Callback functions
    void tick_SCb(builtin_interfaces::msg::Time msg);
    void baro_SCb(sensor_msgs::msg::FluidPressure msg);
    void imu_SCb(sensor_msgs::msg::Imu msg);
    // void mag_SCb();
    // void gps_SCb();

    // Variables to store received data
    double attiW, attiX, attiY, attiZ, angVelX, angVelY, angVelZ, linAccX, linAccY, linAccZ, baroAlt;

    // Variables to store estiated roll, pitch and yaw data
    double roll, pitch, yaw;

    // Complementary filter parameters
    double alf = 0.98;

    // Keep track of last time-stamp
    int64_t imu_lts = 0, baro_lts = 0, tick_lts = 0;

    // Time difference between two samples
    int64_t sampleDt_ns = 0;
};

#endif // __NAVIGATION_NODE_HEADER__