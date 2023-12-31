// Add package headers

// Add ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <quad_sim_interfaces/msg/arm_state.hpp>
#include <quad_sim_interfaces/msg/quad_esc.hpp>

#ifndef __CONTROL_NODE_HEADER__
#define __CONTROL_NODE_HEADER__

class controlNode : public rclcpp::Node
{
public:
    controlNode();

    // Define callback functions for pubs and subs
    void armState_PFn(const int64_t &timeStamp_ns, const bool &armState);
    void motVolts_PFn(const int64_t &timeStamp_ns, const std::vector<double> &currAtt, const std::vector<double> &desAtt);

private:
    // Variables for publishers
    rclcpp::Publisher<quad_sim_interfaces::msg::ArmState>::SharedPtr armState_Pub;
    rclcpp::Publisher<quad_sim_interfaces::msg::QuadESC>::SharedPtr motVolts_Pub;

    // Variables to maintain publication time
    int64_t motVoltsDt_ns = 0;

    // Variables to keep track of last publication time
    int64_t motVolts_lpt = 0;
};

#endif // __CONTROL_NODE_HEADER__