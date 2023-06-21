#include "filter.hpp"

fltr::fltr() : Node("quad_filter")
{
	// Setup publishers and subscribers
	posSub = this->create_subscription<geometry_msgs::msg::Vector3>("/pos", 10, std::bind(&fltr::posSubCb, this, std::placeholders::_1));
	angVelSub = this->create_subscription<geometry_msgs::msg::Vector3>("/angVel", 10, std::bind(&fltr::angVelSubCb, this, std::placeholders::_1));
	linAccSub = this->create_subscription<geometry_msgs::msg::Vector3>("/linAcc", 10, std::bind(&fltr::linAccSubCb, this, std::placeholders::_1));
	cmplFltrPub = this->create_publisher<geometry_msgs::msg::Vector3>("/pose", 10);

	// Initialize the data
	posMsg.x = 0;
	posMsg.y = 0;
	posMsg.z = 0;
	angVelMsg.x = 0;
	angVelMsg.y = 0;
	angVelMsg.z = 0;
	linAccMsg.x = 0;
	linAccMsg.y = 0;
	linAccMsg.z = -1;
	cmplFltrMsg.x = 0;
	cmplFltrMsg.y = 0;
	cmplFltrMsg.z = 0;

	// Send out initial values
	cmplFltrPub->publish(cmplFltrMsg);

	// Set-up the publisher callback, it is assumed that the sensor sampling rate is 0.01s
	imuDt = 0.01;
	alf = 0.02;
	cmplFltrPubFunTmr = this->create_wall_timer(std::chrono::milliseconds(int(imuDt * 1000)), std::bind(&fltr::cmplFltrPubFun, this));
}

void fltr::posSubCb(const geometry_msgs::msg::Vector3 pos)
{
	this->posMsg = pos;
}

void fltr::angVelSubCb(const geometry_msgs::msg::Vector3 angVel)
{
	this->angVelMsg = angVel;
}

void fltr::linAccSubCb(const geometry_msgs::msg::Vector3 linAcc)
{
	this->linAccMsg = linAcc;
}

void fltr::cmplFltrPubFun()
{
	float pitchFromAccel = 0;
	float rollFromAccel = 0;

	// Extract roll and pitch from the accelerometer
	pitchFromAccel = atan2(linAccMsg.x, sqrt(pow(linAccMsg.y, 2) + pow(linAccMsg.z, 2)));
	rollFromAccel = atan2(-linAccMsg.y, sqrt(pow(linAccMsg.x, 2) + pow(linAccMsg.z, 2)));
	// rollFromAccel = atan2(accelerometer.y, accelerometer.z);

	// Complimentary Filter, fuse accelerometer data with gyro integration
	// angle = (1 - alf) * (angle + gyroscope * dt) + alf * accelerometer
	cmplFltrMsg.x = (1 - alf) * (cmplFltrMsg.x + (angVelMsg.x * imuDt)) + alf * rollFromAccel;
	cmplFltrMsg.y = (1 - alf) * (cmplFltrMsg.y + (angVelMsg.y * imuDt)) + alf * pitchFromAccel;

	// Yaw cannot be determined from accelerometer, so only gyro is used
	cmplFltrMsg.z += angVelMsg.z * imuDt;

	// Publish the filtered data
	cmplFltrPub->publish(cmplFltrMsg);

	// The above code returns the angle in RADIANS
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	// Initialize filter node
	rclcpp::Node::SharedPtr filter = std::make_shared<fltr>();

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