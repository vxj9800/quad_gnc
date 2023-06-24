#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "ncurses.h"

#ifndef __QUAD_GNC_KBINPUT__
#define __QUAD_GNC_KBINPUT__

/**
 * @brief This class is a parallel of RC input where the input is taken from a keyboard and the values are published as RC signal.
 * 
*/
class kbInput : public rclcpp::Node
{
public:
    kbInput();

private:
    void kbInputPublish(void);
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr roll_pub, pitch_pub, yaw_pub, thrust_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub;
    std_msgs::msg::UInt16 roll, pitch, yaw, thrust;
    std_msgs::msg::Bool arm;
    rclcpp::TimerBase::SharedPtr timer;
};

#endif // __QUAD_GNC_KBINPUT__