#include "kbInput.hpp"

kbInput::kbInput() : Node("kbInput")
{
	// Create the publishers
	rcRollPub = this->create_publisher<std_msgs::msg::UInt16>("rc/roll", 10);
	rcPitchPub = this->create_publisher<std_msgs::msg::UInt16>("rc/pitch", 10);
	rcYawPub = this->create_publisher<std_msgs::msg::UInt16>("rc/yaw", 10);
	rcThrustPub = this->create_publisher<std_msgs::msg::UInt16>("rc/thrust", 10);
	rcArmPub = this->create_publisher<std_msgs::msg::Bool>("rc/arm", 10);

	// Set default message values
	rcRoll.data = 1500;
	rcPitch.data = 1500;
	rcYaw.data = 1500;
	rcThrust.data = 1000;
	rcArm.data = 0;

	// Publish default values
	rcRollPub->publish(rcRoll);
	rcPitchPub->publish(rcPitch);
	rcYawPub->publish(rcYaw);
	rcThrustPub->publish(rcThrust);
	rcArmPub->publish(rcArm);

	// Create a one-shot timer to call kbInputPublisher function
	timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&kbInput::kbInputPublish, this));
}

void kbInput::kbInputPublish()
{
	// Variables to store keypress values
	static int c, cLast;

	// Get the spring loaded sticks back to default position if it was not pressed last time
	if ((cLast != 'a') && (cLast != 'd'))
		rcYaw.data = 1500;
	if ((cLast != 'j') && (cLast != 'l'))
		rcRoll.data = 1500;
	if ((cLast != 'i') && (cLast != 'k'))
		rcPitch.data = 1500;

	// Look for specific key input and increase the input two times
	switch ((c = getch()))
	{
	case 'w':
		rcThrust.data += (rcThrust.data < 2000) ? 10 : 0;
		break;
	case 'a':
		rcYaw.data += (rcYaw.data < 2000) ? 10 : 0;
		break;
	case 's':
		rcThrust.data -= (rcThrust.data > 1000) ? 10 : 0;
		break;
	case 'd':
		rcYaw.data -= (rcYaw.data > 1000) ? 10 : 0;
		break;
	case 'i':
		rcPitch.data += (rcPitch.data < 2000) ? 10 : 0;
		break;
	case 'j':
		rcRoll.data += (rcRoll.data < 2000) ? 10 : 0;
		break;
	case 'k':
		rcPitch.data -= (rcPitch.data > 1000) ? 10 : 0;
		break;
	case 'l':
		rcRoll.data -= (rcRoll.data > 1000) ? 10 : 0;
		break;
	case 'r':
		if (rcRoll.data == 1500 && rcPitch.data == 1500 && rcYaw.data == 1500 && rcThrust.data == 1000)
			rcArm.data = !rcArm.data;
		break;
	default:
		break;
	}
	// Record last keypress
	cLast = c;

	// Publish the rc signal
	rcThrustPub->publish(rcThrust);
	rcRollPub->publish(rcRoll);
	rcPitchPub->publish(rcPitch);
	rcYawPub->publish(rcYaw);
	rcArmPub->publish(rcArm);

	// Clean-up the input for unwanted key presses
	flushinp();
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	// Setup ncurses
	initscr(); // Start curses mode
	noecho(); // Turn off echoing
	cbreak(); // No carriage return input required
	nodelay(stdscr, true); // Return immediately from getch

	// Initialize kbInput node
	rclcpp::Node::SharedPtr keyboardInput = std::make_shared<kbInput>();

	// Create an executor
	rclcpp::executors::SingleThreadedExecutor exec;

	// Add nodes to the executor
	exec.add_node(keyboardInput);

	// Run the executor
	exec.spin();

	// Cleanup
	rclcpp::shutdown();
	endwin(); // End curses mode
	return 0;
}