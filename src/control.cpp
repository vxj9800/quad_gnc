#include <quad_gnc/control.hpp>

controlNode::controlNode(int64_t motVoltsDt_ns) : Node("controlNode"), motVoltsDt(motVoltsDt_ns / 1000000000, motVoltsDt_ns % 1000000000), motVolts_lpt(0, 0, RCL_ROS_TIME)
{
    // Setup publishers and subscribers
    armState_Pub = create_publisher<quad_sim_interfaces::msg::ArmState>("armed", rclcpp::SensorDataQoS());
    motVolts_Pub = create_publisher<quad_sim_interfaces::msg::QuadESC>("motEsc", rclcpp::SensorDataQoS());
}

void controlNode::armState_PFn(const bool &armState)
{
    // Create message variable
    quad_sim_interfaces::msg::ArmState msg;

    // Add data to the message
    msg.header.stamp = now();
    msg.armed = armState;

    // Publish the message
    armState_Pub->publish(msg);
}

void controlNode::motVolts_PFn(const std::vector<double> &currAtt, const std::vector<double> &desAtt)
{
    // Get current time
    rclcpp::Time timeNow = now();

    // Check if enough time has passed
    if ((motVolts_lpt + motVoltsDt) <= timeNow)
    {
        // Variables to store errors, error rates and error integrals
        static float eR_prev, eP_prev, eY_prev, eT_prev; // Previous roll, pitch, yaw and thrust errors
        float eR, eP, eY, eT;                            // Current roll, pitch, yaw and thrust errors
        static float eR_int, eP_int, eY_int, eT_int;     // Integrated roll, pitch, yaw and thrust errors
        float eR_dot, eP_dot, eY_dot, eT_dot;            // Roll, pitch, yaw and thrust error rates

        // Define PID gains
        float rP = 0.01, pP = 0.01, yP = 0.5, tP = 0.8; // Proportional gains
        float rD = 0.002, pD = 0.002, yD = 0.002, tD = 0;     // Derivative gains
        float rI = 0, pI = 0, yI = 0, tI = 0;             // Integral gains

        // Variables for final values
        float roll = 0, pitch = 0, yaw = 0, thrust = 0;

        // Compute current errors
        eR = desAtt[1] - currAtt[1];
        eP = desAtt[2] - currAtt[2];
        eY = desAtt[3] - currAtt[3];
        eT = desAtt[0];

        // Compute error rates
        eR_dot = (eR - eR_prev) / motVoltsDt.seconds();
        eP_dot = (eP - eP_prev) / motVoltsDt.seconds();
        eY_dot = (eY - eY_prev) / motVoltsDt.seconds();
        eT_dot = (eT - eT_prev) / motVoltsDt.seconds();

        // Integrate error over time
        eR_int += eR * motVoltsDt.seconds();
        eP_int += eP * motVoltsDt.seconds();
        eY_int += eY * motVoltsDt.seconds();
        eT_int += eT * motVoltsDt.seconds();

        // Implement control laws
        thrust = tP * eT;
        roll = rP * eR + rD * eR_dot + rI * eR_int;
        pitch = pP * eP + pD * eP_dot + pI * eP_int;
        yaw = yP * eY + yD * eY_dot + yI * eY_int;

        // Update the last value
        eR_prev = eR;
        eP_prev = eP;
        eY_prev = eY;
        eT_prev = eT;

        // Create message variable
        quad_sim_interfaces::msg::QuadESC msg;

        // Update the message header
        msg.header.stamp = timeNow;

        // Load the data in motVoltages vector
        msg.mot_b = (thrust - pitch + roll + yaw); // Add fl value
        msg.mot_c = (thrust + pitch + roll - yaw); // Add bl value
        msg.mot_d = (thrust + pitch - roll + yaw); // Add br value
        msg.mot_e = (thrust - pitch - roll - yaw); // Add fr value

        // Limit the esc signal values between 0 and 1
        msg.mot_b = (msg.mot_b > 1) ? 1 : msg.mot_b;
        msg.mot_b = (msg.mot_b < 0) ? 0 : msg.mot_b;
        msg.mot_c = (msg.mot_c > 1) ? 1 : msg.mot_c;
        msg.mot_c = (msg.mot_c < 0) ? 0 : msg.mot_c;
        msg.mot_d = (msg.mot_d > 1) ? 1 : msg.mot_d;
        msg.mot_d = (msg.mot_d < 0) ? 0 : msg.mot_d;
        msg.mot_e = (msg.mot_e > 1) ? 1 : msg.mot_e;
        msg.mot_e = (msg.mot_e < 0) ? 0 : msg.mot_e;

        // Assign motVoltages pointer to message and publish the messsage
        motVolts_Pub->publish(msg);

        // Update last publication time
        motVolts_lpt = timeNow;
    }
}