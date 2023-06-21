#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16.hpp"

#ifndef __QUAD_GNC_CONTROLLER__
#define __QUAD_GNC_CONTROLLER__

class cntrl : public rclcpp::Node
{
public:
    cntrl();

private:
    // Publishers, subscribers and variables for quadSim
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr time_sub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motVolts_pub;
    float curr_time = 0, last_time = 0;
    std_msgs::msg::Float32MultiArray motVolts;

    // Subscribers and variables for kbInput publishers
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr roll_sub, pitch_sub, yaw_sub, thrust_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub;
    float roll, pitch, yaw, thrust;
    bool arm;

    // Define callback functions for pubs and subs
    void timeSubCb(const builtin_interfaces::msg::Time timeMsg);
    void rollSubCb(const std_msgs::msg::UInt16 roll);
    void pitchSubCb(const std_msgs::msg::UInt16 pitch);
    void yawSubCb(const std_msgs::msg::UInt16 yaw);
    void thrustSubCb(const std_msgs::msg::UInt16 thrust);
    void armSubCb(const std_msgs::msg::Bool arm);
};

#endif // __QUAD_GNC_CONTROLLER__