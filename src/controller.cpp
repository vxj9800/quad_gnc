#include "controller.hpp"

anglCntrl::anglCntrl() : Node("angleModeControl")
{
	// Setup publishers and subscribers
	timeSub = this->create_subscription<builtin_interfaces::msg::Time>("/time", 10, std::bind(&anglCntrl::timeSubCb, this, std::placeholders::_1));
	motVoltsPub = this->create_publisher<std_msgs::msg::Float32MultiArray>("motVolts", 10);
	rcRollSub = this->create_subscription<std_msgs::msg::UInt16>("rc/roll", 10, std::bind(&anglCntrl::rcRollSubCb, this, std::placeholders::_1));
	rcPitchSub = this->create_subscription<std_msgs::msg::UInt16>("rc/pitch", 10, std::bind(&anglCntrl::rcPitchSubCb, this, std::placeholders::_1));
	rcYawSub = this->create_subscription<std_msgs::msg::UInt16>("rc/yaw", 10, std::bind(&anglCntrl::rcYawSubCb, this, std::placeholders::_1));
	rcThrustSub = this->create_subscription<std_msgs::msg::UInt16>("rc/thrust", 10, std::bind(&anglCntrl::rcThrustSubCb, this, std::placeholders::_1));
	rcArmSub = this->create_subscription<std_msgs::msg::Bool>("rc/arm", 10, std::bind(&anglCntrl::rcArmSubCb, this, std::placeholders::_1));
	cmplFltrRpySub = this->create_subscription<geometry_msgs::msg::Vector3>("imu/attitude", 10, std::bind(&anglCntrl::cmplFltrRpySubCb, this, std::placeholders::_1));

	// Initialize the data
	motVolts.layout.dim.push_back(std_msgs::msg::MultiArrayDimension()); // Add the first dimension
	motVolts.layout.dim[0].label = "fl_bl_br_fr";						// Define order of values, i.e. front-left, back-left, back-right, front-right
	motVolts.layout.dim[0].size = 4;										// Say that there will be 4 values in this dimension
	motVolts.set__data(std::vector<float>(4, 0)); // Set all values to zero
	desRoll = 0;
	desPitch = 0;
	desYaw = 0;
	desThrust = 0;
	armed = false;
	curPitch = 0;
	curRoll = 0;
	curYaw = 0;

	// Send out initial values
	motVoltsPub->publish(motVolts);
}

void anglCntrl::timeSubCb(const builtin_interfaces::msg::Time timeMsg)
{
	// Get the battery voltage
	static float vBat = 16.8; // Copied from MATLAB for now

	// Variables to store errors, error rates and error integrals
	static float eR_prev, eP_prev, eY_prev, eT_prev; // Previous roll, pitch, yaw and thrust errors
	float eR, eP, eY, eT; // Current roll, pitch, yaw and thrust errors
	static float eR_int, eP_int, eY_int, eT_int; // Integrated roll, pitch, yaw and thrust errors
	float eR_dot, eP_dot, eY_dot, eT_dot; // Roll, pitch, yaw and thrust error rates

	// Define PID gains
	float rP = 0.005, pP = 0.005, yP = 0, tP = 0.8; // Proportional gains
	float rD = 0.01, pD = 0.01, yD = 0, tD = 0; // Derivative gains
	float rI = 0, pI = 0, yI = 0, tI = 0; // Integral gains

	// Variables for final values
	float roll = 0, pitch = 0, yaw = 0, thrust = 0;

	// If system is not armed then send zero volts to all motors
	if (!armed)
	{
		motVolts.data[0] = 0; // Add fl value
		motVolts.data[1] = 0; // Add bl value
		motVolts.data[2] = 0; // Add br value
		motVolts.data[3] = 0; // Add fr value

		// Assign motVoltages pointer to message and publish the messsage
		motVoltsPub->publish(motVolts);

		return;
	}

	// Compute current errors
	eR = (desRoll - curRoll) / 2 / rollRange;
	eP = (desPitch - curPitch) / 2 / pitchRange;
	eY = desYaw - curYaw;
	eT = desThrust;

	// Compute error rates
	eR_dot = (eR - eR_prev) / cntrlLoopDt;
	eP_dot = (eP - eP_prev) / cntrlLoopDt;
	eY_dot = (eY - eY_prev) / cntrlLoopDt;
	eT_dot = (eT - eT_prev) / cntrlLoopDt;

	// Integrate error over time
	eR_int += eR * cntrlLoopDt;
	eP_int += eP * cntrlLoopDt;
	eY_int += eY * cntrlLoopDt;
	eT_int += eT * cntrlLoopDt;

	// Implement control laws
	thrust = tP * eT;
	roll = rP * eR + rD * eR_dot + rI * eR_int;
	pitch = pP * eP + pD * eP_dot + pI * eP_int;

	// Update the last value
	eR_prev = eR;
	eP_prev = eP;
	eY_prev = eY;
	eT_prev = eT;

	// Load the data in motVoltages vector
	motVolts.data[0] = (thrust - pitch + roll + yaw) * vBat; // Add fl value
	motVolts.data[1] = (thrust + pitch + roll - yaw) * vBat; // Add bl value
	motVolts.data[2] = (thrust + pitch - roll + yaw) * vBat; // Add br value
	motVolts.data[3] = (thrust - pitch - roll - yaw) * vBat; // Add fr value

	// Limit the voltage values
	for (int i = 0; i < 4; ++i)
	{
		motVolts.data[i] = (motVolts.data[i] > vBat) ? vBat : motVolts.data[i];
		motVolts.data[i] = (motVolts.data[i] < 0) ? 0 : motVolts.data[i];
	}

	// Assign motVoltages pointer to message and publish the messsage
	motVoltsPub->publish(motVolts);
}

void anglCntrl::rcRollSubCb(std_msgs::msg::UInt16 roll)
{
	// Desired roll values should fall between -rollRange and rollRange
	this->desRoll = (roll.data - 1500) / 500.0 * rollRange;
}

void anglCntrl::rcPitchSubCb(std_msgs::msg::UInt16 pitch)
{
	// Desired pitch values should fall between -pitchRange and pitchRange
	this->desPitch = (pitch.data - 1500) / 500.0 * pitchRange;
}

void anglCntrl::rcYawSubCb(std_msgs::msg::UInt16 yaw)
{
	// Desired yaw values should fall between -1 and 1
	this->desYaw = (yaw.data - 1500) / 500.0;
}

void anglCntrl::rcThrustSubCb(std_msgs::msg::UInt16 thrust)
{
	// Desired thrust values should fall between 0 and 1
	this->desThrust = (thrust.data - 1000) / 1000.0;
}

void anglCntrl::rcArmSubCb(std_msgs::msg::Bool arm)
{
	// armed value should either be 0/false or 1/true
	this->armed = arm.data;
}

void anglCntrl::cmplFltrRpySubCb(const geometry_msgs::msg::Vector3 rpy)
{
	// Update current roll, pitch and yaw values
	this->curRoll = rpy.x;
	this->curPitch = rpy.y;
	this->curYaw = rpy.z;
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	// Initialize controller node
	rclcpp::Node::SharedPtr controller = std::make_shared<anglCntrl>();

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