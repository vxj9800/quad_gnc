#include <functional>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#ifndef __QUAD_GNC_FILTER__
#define __QUAD_GNC_FILTER__

class fltr : public rclcpp::Node
{
public:
    fltr();

private:
    // Publishers, subscribers and variables for quadSim
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr posSub, angVelSub, linAccSub;
    geometry_msgs::msg::Vector3 posMsg, angVelMsg, linAccMsg;
    double imuDt, alf; // IMU sampling rate

    // Publishers for controller node
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr cmplFltrPub;
    rclcpp::TimerBase::SharedPtr cmplFltrPubFunTmr;
    geometry_msgs::msg::Vector3 cmplFltrMsg;

    // Define callback functions for pubs and subs
    void posSubCb(const geometry_msgs::msg::Vector3 pos);
    void angVelSubCb(const geometry_msgs::msg::Vector3 angVel);
    void linAccSubCb(const geometry_msgs::msg::Vector3 linAcc);
    void cmplFltrPubFun();
};

#endif // __QUAD_GNC_FILTER__