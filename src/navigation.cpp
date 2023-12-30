#include <quad_gnc/navigation.hpp>

navigationNode::navigationNode() : Node("navigationNode")
{
    // Initialize the subscribers
    tick_Sub = create_subscription<builtin_interfaces::msg::Time>("tick", rclcpp::SensorDataQoS(), std::bind(&navigationNode::tick_Scb, this, std::placeholders::_1));
    imu_Sub = create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS(), std::bind(&navigationNode::imu_SCb, this, std::placeholders::_1));
    baro_Sub = create_subscription<sensor_msgs::msg::FluidPressure>("baro", rclcpp::SensorDataQoS(), std::bind(&navigationNode::baro_SCb, this, std::placeholders::_1));
}

void navigationNode::baro_SCb(sensor_msgs::msg::FluidPressure msg)
{
    double p = msg.fluid_pressure / 1000; // Pressure in kPa

    // Assuming that the quad will always be in Troposphere,
    // the altitude can be computed as
    baroAlt = 44398 * (1 - pow(p / 101.29, 1 / 5.256));
}

void navigationNode::imu_SCb(sensor_msgs::msg::Imu msg)
{
    angVelX = msg.angular_velocity.x;
    angVelY = msg.angular_velocity.y;
    angVelZ = msg.angular_velocity.z;

    linAccX = msg.linear_acceleration.x;
    linAccY = msg.linear_acceleration.y;
    linAccZ = msg.linear_acceleration.z;

    // Time duration for the new data
    int64_t newTime = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec;
    int64_t dt = newTime - imu_lts;
    imu_lts = dt ? newTime : imu_lts;

    // Estimate body orientation
    double pitchFromAccel = 0;
    double rollFromAccel = 0;

    // Extract roll and pitch from the accelerometer
    // These formulae assume that z-axis point away from the earth
    pitchFromAccel = atan2(linAccX, sqrt(pow(linAccY, 2) + pow(linAccZ, 2)));
    rollFromAccel = atan2(-linAccY, sqrt(pow(linAccX, 2) + pow(linAccZ, 2)));
    // rollFromAccel = atan2(accelerometer.y, accelerometer.z);

    // Complimentary Filter, fuse accelerometer data with gyro integration
    // angle = alf * (angle + gyroscope * dt) + (1 - alf) * accelerometer
    roll = alf * (roll + (angVelX * dt)) + (1 - alf) * rollFromAccel;
    pitch = alf * (pitch + (angVelY * dt)) + (1 - alf) * pitchFromAccel;

    // Yaw cannot be determined from accelerometer, so only gyro is used
    yaw += angVelY * dt;

    // The above code returns the angle in RADIANS
}

void navigationNode::tick_Scb(builtin_interfaces::msg::Time msg)
{
    // Eventually, perform more complex filtering using all the sensor data.
    // That will need all the sensor data to be received first.
    // The code here will also say that new data is received and processed,
    // so that the full gnc code can read the estimated values and pass
    // those to the controller.

    // Update the last data received time
    tick_lts = msg.sec * 1000000000 + msg.nanosec;

    // For now, just set the new data variable as true.
    newDataAvailable = true;
}

bool navigationNode::getNewData(double &roll, double &pitch, double &yaw)
{
    if (newDataAvailable)
    {
        roll = this->roll;
        pitch = this->pitch;
        yaw = this->yaw;
    }
    return newDataAvailable;
}