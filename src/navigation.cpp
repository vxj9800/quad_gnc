#include <quad_gnc/navigation.hpp>

navigationNode::navigationNode() : Node("navigationNode"), imu_lts(0, 0, RCL_ROS_TIME), baro_lts(0, 0, RCL_ROS_TIME), imuDt(0, 1e6), baroDt(0, 1e6)
{
    // Initialize the subscribers
    imu_Sub = create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS(), std::bind(&navigationNode::imu_SCb, this, std::placeholders::_1));
    baro_Sub = create_subscription<sensor_msgs::msg::FluidPressure>("baro", rclcpp::SensorDataQoS(), std::bind(&navigationNode::baro_SCb, this, std::placeholders::_1));
}

void navigationNode::baro_SCb(sensor_msgs::msg::FluidPressure msg)
{
    // Update time values
    baroDt = rclcpp::Time(msg.header.stamp) - baro_lts;
    baro_lts = msg.header.stamp;

    // Update sensor value
    double p = msg.fluid_pressure / 1000; // Pressure in kPa

    // Assuming that the quad will always be in Troposphere,
    // the altitude can be computed as
    baroAlt = 44398 * (1 - pow(p / 101.29, 1 / 5.256));
}

void navigationNode::imu_SCb(sensor_msgs::msg::Imu msg)
{
    // Update time values
    imuDt = rclcpp::Time(msg.header.stamp) - imu_lts;
    imu_lts = msg.header.stamp;

    // Update angular velocity
    angVelX = msg.angular_velocity.x;
    angVelY = msg.angular_velocity.y;
    angVelZ = msg.angular_velocity.z;

    // Update linear acceleration
    linAccX = msg.linear_acceleration.x;
    linAccY = msg.linear_acceleration.y;
    linAccZ = msg.linear_acceleration.z;

    // Estimate body orientation
    double pitchFromAccel = 0;
    double rollFromAccel = 0;

    // Extract roll and pitch from the accelerometer
    // These formulae assume that z-axis point away from the earth
    pitchFromAccel = atan2(linAccX, sqrt(pow(linAccY, 2) + pow(linAccZ, 2)));
    rollFromAccel = atan2(-linAccY, sqrt(pow(linAccX, 2) + pow(linAccZ, 2)));
    // rollFromAccel = atan2(linAccY, linAccZ);

    // Complimentary Filter, fuse accelerometer data with gyro integration
    // angle = alf * (angle + gyroscope * dt) + (1 - alf) * accelerometer
    roll = alf * (roll + (angVelX * imuDt.seconds())) + (1 - alf) * rollFromAccel;
    pitch = alf * (pitch + (angVelY * imuDt.seconds())) + (1 - alf) * pitchFromAccel;

    // For current controller implementation, the yaw rate matters only
    yaw = angVelZ;
}

void navigationNode::getNewEstimate(double &roll, double &pitch, double &yaw)
{
    roll = this->roll;
    pitch = this->pitch;
    yaw = this->yaw;
}