#include "kbInput.hpp"

kbInput::kbInput() : Node("kbInput")
{
	// Create the publishers
	roll_pub = this->create_publisher<std_msgs::msg::UInt16>("roll", 10);
	pitch_pub = this->create_publisher<std_msgs::msg::UInt16>("pitch", 10);
	yaw_pub = this->create_publisher<std_msgs::msg::UInt16>("yaw", 10);
	thrust_pub = this->create_publisher<std_msgs::msg::UInt16>("thrust", 10);
	arm_pub = this->create_publisher<std_msgs::msg::Bool>("arm", 10);

	// Set default message values
	roll.data = 1500;
	pitch.data = 1500;
	yaw.data = 1500;
	thrust.data = 1000;
	arm.data = 0;

	// Publish default values
	roll_pub->publish(roll);
	pitch_pub->publish(pitch);
	yaw_pub->publish(yaw);
	thrust_pub->publish(thrust);
	arm_pub->publish(arm);

	// Create a one-shot timer to call kbInputPublisher function
	timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&kbInput::kbInputPublish, this));
}

void kbInput::kbInputPublish()
{

	// Break condition variable
	static bool ctrlC = 0;

	// Reduce the values by one
	thrust.data -= (thrust.data > 1000) ? 10 : 0;
	yaw.data -= (yaw.data > 1500) ? 10 : 0;
	yaw.data += (yaw.data < 1500) ? 10 : 0;
	roll.data -= (roll.data > 1500) ? 10 : 0;
	roll.data += (roll.data < 1500) ? 10 : 0;
	pitch.data -= (pitch.data > 1500) ? 10 : 0;
	pitch.data += (pitch.data < 1500) ? 10 : 0;

	// Look for specific key input and increase the input two times
	if (!ctrlC)
	{
		switch (getch())
		{
		case 'w':
			thrust.data += (thrust.data < 2000) ? 20 : 0;
			break;
		case 'a':
			yaw.data += (yaw.data < 2000) ? 20 : 0;
			break;
		case 's':
			thrust.data -= (thrust.data > 1000) ? 20 : 0;
			break;
		case 'd':
			yaw.data -= (yaw.data > 1000) ? 20 : 0;
			break;
		case 'i':
			pitch.data += (pitch.data < 2000) ? 20 : 0;
			break;
		case 'j':
			roll.data -= (roll.data < 2000) ? 20 : 0;
			break;
		case 'k':
			pitch.data -= (pitch.data > 1000) ? 20 : 0;
			break;
		case 'l':
			roll.data += (roll.data > 1000) ? 20 : 0;
			break;
		case 'r':
			if (roll.data == 1500 && pitch.data == 1500 && yaw.data == 1500 && thrust.data == 1000)
			{
				arm.data = !arm.data;
			}
			else
			{
				printw("Can't arm. roll: %d, pitch: %d, yaw: %d, thrust: %d\r\n", roll.data, pitch.data, yaw.data, thrust.data);
				refresh();
			}
			break;
		case 'q': // Ctrl+C
			ctrlC = true;
			break;
		default:
			break;
		}
		thrust_pub->publish(thrust);
		roll_pub->publish(roll);
		pitch_pub->publish(pitch);
		yaw_pub->publish(yaw);
		arm_pub->publish(arm);
		flushinp();
	}
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