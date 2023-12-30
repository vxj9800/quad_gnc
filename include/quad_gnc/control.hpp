// Add package headers

// Add ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <quad_sim_interfaces/msg/arm_state.hpp>

#ifndef __CONTROL_NODE_HEADER__
#define __CONTROL_NODE_HEADER__

class controlNode : public rclcpp::Node
{
public:
    controlNode();

    // Define callback functions for pubs and subs
    void armState_PFn(const int64_t &timeStamp_ns, const bool &armState);

private:
    // Variables for publishers
    rclcpp::Publisher<quad_sim_interfaces::msg::ArmState>::SharedPtr armState_Pub;
};

#endif // __CONTROL_NODE_HEADER__