#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#ifndef __QUAD_GNC_CONTROLLER__
#define __QUAD_GNC_CONTROLLER__

class anglCntrl : public rclcpp::Node
{
public:
    anglCntrl();

private:
    // Publishers, subscribers and variables for quadSim
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr timeSub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motVoltsPub;
    std_msgs::msg::Float32MultiArray motVolts;

    // Subscribers and variables for RC values
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr rcRollSub, rcPitchSub, rcYawSub, rcThrustSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rcArmSub;
    float desRoll, desPitch, desYaw, desThrust;
    bool armed;

    // Subscribers and variables for complementary filter values
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmplFltrRpySub;
    float curRoll, curPitch, curYaw;

    // Other variables
    float cntrlLoopDt = 0.01;                            // Simulation frame period, taken from MATLAB
    float rollRange = M_PI / 10, pitchRange = M_PI / 10; // Range for desired roll and pitch angles, set to ~18Deg for now

    // Define callback functions for pubs and subs
    void
    timeSubCb(const builtin_interfaces::msg::Time timeMsg);
    void rcRollSubCb(const std_msgs::msg::UInt16 rcRoll);
    void rcPitchSubCb(const std_msgs::msg::UInt16 rcPitch);
    void rcYawSubCb(const std_msgs::msg::UInt16 rcYaw);
    void rcThrustSubCb(const std_msgs::msg::UInt16 rcThrust);
    void rcArmSubCb(const std_msgs::msg::Bool rcArm);
    void cmplFltrRpySubCb(const geometry_msgs::msg::Vector3 rpy);
};

#endif // __QUAD_GNC_CONTROLLER__