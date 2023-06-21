#include "controller.hpp"
#include "kbInput.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Initialize controller node
    rclcpp::Node::SharedPtr controller = std::make_shared<cntrl>();
    rclcpp::Node::SharedPtr keyboardInput = std::make_shared<kbInput>();

    // Create an executor
    rclcpp::executors::MultiThreadedExecutor exec;

    // Add nodes to the executor
    exec.add_node(controller);
    exec.add_node(keyboardInput);

    // Run the executor
    exec.spin();

    // Cleanup
    rclcpp::shutdown();
    return 0;
}