#include <functional>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#ifndef __QUAD_GNC_FILTER__
#define __QUAD_GNC_FILTER__

class cmplFltr : public rclcpp::Node
{
public:
    cmplFltr();

private:
    // Publishers, subscribers and variables for quadSim
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imuGyroRawSub, imuAcclRawSub;
    geometry_msgs::msg::Vector3 imuGyroRawMsg, imuAcclRawMsg;
    double imuDt, alf; // IMU sampling rate

    // Publishers for controller node
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr cmplFltrAttitudePub;
    rclcpp::TimerBase::SharedPtr cmplFltrAttitudePubFunTmr;
    geometry_msgs::msg::Vector3 cmplFltrAttitudeMsg;

    // Define callback functions for pubs and subs
    void imuGyroRawSubCb(const geometry_msgs::msg::Vector3 imuGyroRaw);
    void imuAcclRawSubCb(const geometry_msgs::msg::Vector3 imuAcclRaw);
    void cmplFltrAttitudePubFun();
};

#endif // __QUAD_GNC_FILTER__