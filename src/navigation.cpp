#include <quad_gnc/navigation.hpp>

navigationNode::navigationNode() : Node("navigationNode")
{
    // Initialize the subscribers
    imu_Sub = create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS(), std::bind(&navigationNode::imu_SCb, this, std::placeholders::_1));
    baro_Sub = create_subscription<sensor_msgs::msg::FluidPressure>("baro", rclcpp::SensorDataQoS(), std::bind(&navigationNode::baro_SCb, this, std::placeholders::_1));
}

void navigationNode::baro_SCb(sensor_msgs::msg::FluidPressure msg)
{
    // Update time values
    baroDt_ns = msg.header.stamp.sec * (int64_t)1000000000 + msg.header.stamp.nanosec - baro_lts;
    baro_lts += baroDt_ns;

    // Update sensor value
    double p = msg.fluid_pressure / 1000; // Pressure in kPa

    // Assuming that the quad will always be in Troposphere,
    // the altitude can be computed as
    baroAlt = 44398 * (1 - pow(p / 101.29, 1 / 5.256));
}

void navigationNode::imu_SCb(sensor_msgs::msg::Imu msg)
{
    // Update time values
    imuDt_ns = msg.header.stamp.sec * (int64_t)1000000000 + msg.header.stamp.nanosec - imu_lts;
    imu_lts += imuDt_ns;

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
    pitchFromAccel = atan2(-linAccX, sqrt(pow(linAccY, 2) + pow(linAccZ, 2)));
    rollFromAccel = atan2(linAccY, sqrt(pow(linAccX, 2) + pow(linAccZ, 2)));
    // rollFromAccel = atan2(linAccY, linAccZ);

    // Complimentary Filter, fuse accelerometer data with gyro integration
    // angle = alf * (angle + gyroscope * dt) + (1 - alf) * accelerometer
    roll = alf * (roll + (angVelX * imuDt_ns * 1e-9)) + (1 - alf) * rollFromAccel;
    pitch = alf * (pitch + (angVelY * imuDt_ns * 1e-9)) + (1 - alf) * pitchFromAccel;

    // Yaw cannot be determined from accelerometer, so only gyro is used
    yaw += angVelZ * imuDt_ns * 1e-9;

    std::cout << roll << '\t' << pitch << '\t' << yaw << std::endl;

    // // Convert quaternion to euler angles to tune controller
    // // roll (x-axis rotation)
    // double sinr_cosp = 2 * (msg.orientation.w * msg.orientation.x + msg.orientation.y * msg.orientation.z);
    // double cosr_cosp = 1 - 2 * (msg.orientation.x * msg.orientation.x + msg.orientation.y * msg.orientation.y);
    // roll = std::atan2(sinr_cosp, cosr_cosp);

    // // pitch (y-axis rotation)
    // double sinp = std::sqrt(1 + 2 * (msg.orientation.w * msg.orientation.y - msg.orientation.x * msg.orientation.z));
    // double cosp = std::sqrt(1 - 2 * (msg.orientation.w * msg.orientation.y - msg.orientation.x * msg.orientation.z));
    // pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // // yaw (z-axis rotation)
    // double siny_cosp = 2 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y);
    // double cosy_cosp = 1 - 2 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z);
    // yaw = std::atan2(siny_cosp, cosy_cosp);
}

void navigationNode::getNewEstimate(double &roll, double &pitch, double &yaw)
{
    roll = this->roll;
    pitch = this->pitch;
    yaw = this->yaw;
}

int64_t navigationNode::getTimeStamp()
{
    // For now, return imu time-stamp since it is supposed to work at the highest rate
    return imu_lts;
}