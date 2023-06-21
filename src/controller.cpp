#include "controller.hpp"

cntrl::cntrl() : Node("quad_control")
{
	// Setup publishers and subscribers
	time_sub = this->create_subscription<builtin_interfaces::msg::Time>("/time", 10, std::bind(&cntrl::timeSubCb, this, std::placeholders::_1));
	motVolts_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("motVolts", 10);
	roll_sub = this->create_subscription<std_msgs::msg::UInt16>("/roll", 10, std::bind(&cntrl::rollSubCb, this, std::placeholders::_1));
	pitch_sub = this->create_subscription<std_msgs::msg::UInt16>("/pitch", 10, std::bind(&cntrl::pitchSubCb, this, std::placeholders::_1));
	yaw_sub = this->create_subscription<std_msgs::msg::UInt16>("/yaw", 10, std::bind(&cntrl::yawSubCb, this, std::placeholders::_1));
	thrust_sub = this->create_subscription<std_msgs::msg::UInt16>("/thrust", 10, std::bind(&cntrl::thrustSubCb, this, std::placeholders::_1));
	arm_sub = this->create_subscription<std_msgs::msg::Bool>("/arm", 10, std::bind(&cntrl::armSubCb, this, std::placeholders::_1));

	// Initialize the data
	motVolts.layout.dim.push_back(std_msgs::msg::MultiArrayDimension()); // Add the first dimension
	motVolts.layout.dim[0].label = "fl_bl_br_fr";						// Define order of values, i.e. front-left, back-left, back-right, front-right
	motVolts.layout.dim[0].size = 4;										// Say that there will be 4 values in this dimension
	motVolts.set__data(std::vector<float>(4, 0)); // Set all values to zero
	roll = 0;
	pitch = 0;
	yaw = 0;
	thrust = 0;
	arm = false;

	// Send out initial values
	motVolts_pub->publish(motVolts);
}

void cntrl::timeSubCb(const builtin_interfaces::msg::Time timeMsg)
{
	// Update the timestamp
	last_time = curr_time;
	curr_time = timeMsg.sec + timeMsg.nanosec / 1e9f;

	// Get the battery voltage
	float vBat = 16.8; // Copied from MATLAB for now

	// Check if arming is done
	if (!arm)
	{
		roll = 0;
		pitch = 0;
		yaw = 0;
		thrust = 0;
	}

	// Load the data in motVoltages vector
	motVolts.data[0] = (thrust - pitch + roll + yaw) * vBat; // Add fl value
	motVolts.data[1] = (thrust + pitch + roll - yaw) * vBat; // Add bl value
	motVolts.data[2] = (thrust + pitch - roll + yaw) * vBat; // Add br value
	motVolts.data[3] = (thrust - pitch - roll - yaw) * vBat; // Add fr value

	// Assign motVoltages pointer to message and publish the messsage
	motVolts_pub->publish(motVolts);
}

void cntrl::rollSubCb(std_msgs::msg::UInt16 roll)
{
	// roll values should fall between -1 and 1
	this->roll = (roll.data - 1500) / 500.0;
}

void cntrl::pitchSubCb(std_msgs::msg::UInt16 pitch)
{
	// pitch values should fall between -1 and 1
	this->pitch = (pitch.data - 1500) / 500.0;
}

void cntrl::yawSubCb(std_msgs::msg::UInt16 yaw)
{
	// yaw values should fall between -1 and 1
	this->yaw = (yaw.data - 1500) / 500.0;
}

void cntrl::thrustSubCb(std_msgs::msg::UInt16 thrust)
{
	// thrust values should fall between 0 and 1
	this->thrust = (thrust.data - 1000) / 1000.0;
}

void cntrl::armSubCb(std_msgs::msg::Bool arm)
{
	// arm value should either be 0 or 1
	this->arm = arm.data;
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	// Initialize controller node
	rclcpp::Node::SharedPtr controller = std::make_shared<cntrl>();

	// Create an executor
	rclcpp::executors::MultiThreadedExecutor exec;

	// Add nodes to the executor
	exec.add_node(controller);

	// Run the executor
	exec.spin();

	// Cleanup
	rclcpp::shutdown();
	return 0;
}