#include <quad_gnc/control.hpp>

controlNode::controlNode() : Node("controlNode")
{
    // Setup publishers and subscribers
    armState_Pub = create_publisher<quad_sim_interfaces::msg::ArmState>("armed", rclcpp::SensorDataQoS());
}

void controlNode::armState_PFn(const int64_t &timeStamp, const bool &armState)
{
    // Create message variable
    quad_sim_interfaces::msg::ArmState msg;

    // Add data to the message
    msg.header.stamp.sec = timeStamp / 1000000000;
    msg.header.stamp.nanosec = timeStamp % 1000000000;
    msg.armed = armState;

    // Publish the message
    armState_Pub->publish(msg);
}