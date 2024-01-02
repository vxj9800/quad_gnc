# quad_gnc

This is a ROS2 Package that gets sensor data from [`quad_sim`](https://github.com/vxj9800/quad_sim) and user input through a joystick to send out motor voltage values back to the [`quad_sim`](https://github.com/vxj9800/quad_sim).

The `src\guidance.cpp` handles reading of joystick events which are then converted to desired throttle, roll, pitch and yaw-rate values. The joystick axis mapping is as follows, 0: yaw-rate, 1:throttle, 2:roll and 3:pitch. Apart from these, buttons 0 and 1 are used to send stop and run commands to the [`quad_sim`](https://github.com/vxj9800/quad_sim). The limits on roll and pitch are hard-coded in the `Joystick` class header in `include\quad_gnc\guidance.hpp`.

The `src\navigation.cpp` subscribes to the sensor topics from [`quad_sim`](https://github.com/vxj9800/quad_sim) and estimates current roll and pitch. It uses a complementary filter to fuse accelerometer and gyroscope data. Better estimators like EKF can be implemented later.

The `src\control.cpp` implements a simple PID controller to calculate control signal values for the four motors available on the quadcopter. The computed values are published so that [`quad_sim`](https://github.com/vxj9800/quad_sim) can subscribe to those and calculate appropriate motor torque values. This node also publishes a boolean value on `armed` topic to specify whether the simulation should run or not.