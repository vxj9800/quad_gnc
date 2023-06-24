#include "cmplFltr.hpp"

cmplFltr::cmplFltr() : Node("complementaryFilter")
{
	// Setup publishers and subscribers
	imuGyroRawSub = this->create_subscription<geometry_msgs::msg::Vector3>("imu/gyroRaw", 10, std::bind(&cmplFltr::imuGyroRawSubCb, this, std::placeholders::_1));
	imuAcclRawSub = this->create_subscription<geometry_msgs::msg::Vector3>("imu/acclRaw", 10, std::bind(&cmplFltr::imuAcclRawSubCb, this, std::placeholders::_1));
	cmplFltrPosePub = this->create_publisher<geometry_msgs::msg::Vector3>("cmplFltr/pose", 10);

	// Initialize the data
	imuGyroRawMsg.x = 0;
	imuGyroRawMsg.y = 0;
	imuGyroRawMsg.z = 0;
	imuAcclRawMsg.x = 0;
	imuAcclRawMsg.y = 0;
	imuAcclRawMsg.z = -1;
	cmplFltrPoseMsg.x = 0;
	cmplFltrPoseMsg.y = 0;
	cmplFltrPoseMsg.z = 0;

	// Send out initial values
	cmplFltrPosePub->publish(cmplFltrPoseMsg);

	// Set-up the publisher callback, it is assumed that the sensor sampling rate is 0.01s
	imuDt = 0.01;
	alf = 0.02;
	cmplFltrPosePubFunTmr = this->create_wall_timer(std::chrono::milliseconds(int(imuDt * 1000)), std::bind(&cmplFltr::cmplFltrPosePubFun, this));
}

void cmplFltr::imuGyroRawSubCb(const geometry_msgs::msg::Vector3 imuGyroRaw)
{
	this->imuGyroRawMsg = imuGyroRaw;
}

void cmplFltr::imuAcclRawSubCb(const geometry_msgs::msg::Vector3 imuAcclRaw)
{
	this->imuAcclRawMsg = imuAcclRaw;
}

void cmplFltr::cmplFltrPosePubFun()
{
	float pitchFromAccel = 0;
	float rollFromAccel = 0;

	// Extract roll and pitch from the accelerometer
	// These formulae assume that z-axis point away from the earth
	pitchFromAccel = atan2(imuAcclRawMsg.x, sqrt(pow(imuAcclRawMsg.y, 2) + pow(imuAcclRawMsg.z, 2)));
	rollFromAccel = atan2(-imuAcclRawMsg.y, sqrt(pow(imuAcclRawMsg.x, 2) + pow(imuAcclRawMsg.z, 2)));
	// rollFromAccel = atan2(accelerometer.y, accelerometer.z);

	// Complimentary Filter, fuse accelerometer data with gyro integration
	// angle = (1 - alf) * (angle + gyroscope * dt) + alf * accelerometer
	cmplFltrPoseMsg.x = (1 - alf) * (cmplFltrPoseMsg.x + (imuGyroRawMsg.x * imuDt)) + alf * rollFromAccel;
	cmplFltrPoseMsg.y = (1 - alf) * (cmplFltrPoseMsg.y + (imuGyroRawMsg.y * imuDt)) + alf * pitchFromAccel;

	// Yaw cannot be determined from accelerometer, so only gyro is used
	cmplFltrPoseMsg.z += imuGyroRawMsg.z * imuDt;

	// Publish the filtered data
	cmplFltrPosePub->publish(cmplFltrPoseMsg);

	// The above code returns the angle in RADIANS
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	// Initialize filter node
	rclcpp::Node::SharedPtr filter = std::make_shared<cmplFltr>();

	// Create an executor
	rclcpp::executors::MultiThreadedExecutor exec;

	// Add nodes to the executor
	exec.add_node(filter);

	// Run the executor
	exec.spin();

	// Cleanup
	rclcpp::shutdown();
	return 0;
}