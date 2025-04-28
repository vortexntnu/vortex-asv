# Thruster Interface

This package provides the interface between high-level thruster commands and the physical actuators of the ASV.

## Node

The `thruster_interface_asv_node` acts as an interface between **high-level thruster commands** and **low-level PWM signals**.  
It converts force values into appropriate PWM outputs based on thruster characteristics and configuration.

### Goal

- **Receives thrust force commands** for each individual thruster via the [`/freya/thruster_forces`](../thrust_allocator_asv/README.md#freyathruster_forces) topic.
- **Converts the force commands into PWM values** using a hardware-specific mapping.
- **Sends PWM signals** to the motor controller via I2C communication.
- **Applies direction, offset, and saturation limits** based on ROS 2 parameters.
- **Periodically publishes** the PWM values on a ROS 2 topic for debugging and monitoring.
- Acts as the **final interface** between the ROS 2 control system and the physical thrusters of the ASV.

### Subscribers

- [`/freya/thruster_forces`](../thrust_allocator_asv/README.md#freyathruster_forces)

### Publishers

- [`/freya/pwm_output`](#freyapwm_output)

## Topics

- ### `/freya/pwm_output`
  
  | Topic Info         |                                  |
  |--------------------|----------------------------------|
  | **Message type**   | [`std_msgs/msg/Int16MultiArray`](https://docs.ros2.org/foxy/api/std_msgs/msg/Int16MultiArray.html) |
  | **Published by**   | `thruster_interface_asv_node` |
  | **Subscribed by**  | *None* |
  
  ##### Function
  
  Publishes the **PWM values** sent to the ASV’s physical thrusters.  
  Although the PWM signals are actually transmitted to the motor controller via *I2C*, this topic provides a **copy of the signal values for diagnostic purposes**.  
  It allows developers and operators to **monitor the output** of the low-level actuation interface in real time.

- ### [`/freya/thruster_forces`](../thrust_allocator_asv/README.md#freyathruster_forces)
