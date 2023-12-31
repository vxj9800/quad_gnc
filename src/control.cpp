#include <quad_gnc/control.hpp>

controlNode::controlNode() : Node("controlNode")
{
    // Setup publishers and subscribers
    armState_Pub = create_publisher<quad_sim_interfaces::msg::ArmState>("armed", rclcpp::SensorDataQoS());
}

void controlNode::armState_PFn(const int64_t &timeStamp, const bool &armState)
{
    // Create message variable
    quad_sim_interfaces::msg::ArmState msg;

    // Add data to the message
    msg.header.stamp.sec = timeStamp / 1000000000;
    msg.header.stamp.nanosec = timeStamp % 1000000000;
    msg.armed = armState;

    // Publish the message
    armState_Pub->publish(msg);
}

void controlNode::motVolts_PFn(const int64_t &timeStamp_ns, const std::vector<double> &currAtt, const std::vector<double> &desAtt)
{
    // Check if enough time has passed
    if ((timeStamp_ns - motVolts_lpt) >= motVoltsDt_ns)
    {
        // Variables to store errors, error rates and error integrals
        static float eR_prev, eP_prev, eY_prev, eT_prev; // Previous roll, pitch, yaw and thrust errors
        float eR, eP, eY, eT;                            // Current roll, pitch, yaw and thrust errors
        static float eR_int, eP_int, eY_int, eT_int;     // Integrated roll, pitch, yaw and thrust errors
        float eR_dot, eP_dot, eY_dot, eT_dot;            // Roll, pitch, yaw and thrust error rates

        // Define PID gains
        float rP = 0.0005, pP = 0.0005, yP = 0, tP = 0.8; // Proportional gains
        float rD = 0.001, pD = 0.001, yD = 0, tD = 0;     // Derivative gains
        float rI = 0, pI = 0, yI = 0, tI = 0;             // Integral gains

        // Variables for final values
        float roll = 0, pitch = 0, yaw = 0, thrust = 0;

        // Compute current errors
        eR = desAtt[1] - currAtt[1];
        eP = desAtt[2] - currAtt[2];
        eY = desAtt[3] - currAtt[3];
        eT = desAtt[0];

        // Compute error rates
        eR_dot = (eR - eR_prev) / motVoltsDt_ns;
        eP_dot = (eP - eP_prev) / motVoltsDt_ns;
        eY_dot = (eY - eY_prev) / motVoltsDt_ns;
        eT_dot = (eT - eT_prev) / motVoltsDt_ns;

        // Integrate error over time
        eR_int += eR * motVoltsDt_ns;
        eP_int += eP * motVoltsDt_ns;
        eY_int += eY * motVoltsDt_ns;
        eT_int += eT * motVoltsDt_ns;

        // Implement control laws
        thrust = tP * eT;
        roll = rP * eR + rD * eR_dot + rI * eR_int;
        pitch = pP * eP + pD * eP_dot + pI * eP_int;

        // Update the last value
        eR_prev = eR;
        eP_prev = eP;
        eY_prev = eY;
        eT_prev = eT;

        // Create message variable
        quad_sim_interfaces::msg::QuadESC msg;

        // Update the message header
        msg.header.stamp.sec = timeStamp_ns / 1000000000;
        msg.header.stamp.nanosec = timeStamp_ns % 1000000000;

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
        motVolts_lpt = timeStamp_ns;
    }
}