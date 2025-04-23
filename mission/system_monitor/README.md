# System monitor
This package contains a monitoring node for detecting network-related system failures on the ASV

## Node
The `system_monitor_node` implements a system-level safety monitor that checks network connectivity via IP pinging and triggers an emergency shutdown when devices become unreachable.

### Goal
- **Periodically checks** the network connectivity of critical onboard devices via IP address pinging.
- **Detects system-level failures** by monitoring the responsiveness of all listed IPs.
- **Triggers an emergency shutdown** if any device becomes unreachable:
  - Sends a lifecycle transition request to deactivate the [`thrust_allocator_asv_node`](https://github.com/vortexntnu/vortex-asv/blob/main/motion/thrust_allocator_asv/README.md)
  - Publishes a zero-force command to [`/freya/thruster_forces`](#freyathruster_forces)
  - Terminates its execution, requiring **manual intervention** for recovery

### Subscribers
- *None*

### Publishers
- [`/freya/thruster_forces`](#freyathruster_forces)

## Topics
- ### /freya/thruster_forces
  
  | Topic Info         |                                  |
  |--------------------|----------------------------------|
  | **Message type**   | [`std_msgs/msg/Float64MultiArray`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float64MultiArray.html) |
  | **Published by**   | [`thrust_allocator_asv_node`](https://github.com/vortexntnu/vortex-asv/blob/main/motion/thrust_allocator_asv/README.md) <br> [`system_monitor_node`](#node) |
  | **Subscribed by**  | [`thruster_interface_asv_node`](...) |

  #### Function  
  Transmits the **individual force values** to be applied to each thruster of the ASV.  
  These forces are calculated by the [`thrust_allocator_asv_node`](https://github.com/vortexntnu/vortex-asv/blob/main/motion/thrust_allocator_asv/README.md) based on the desired motion commands and the vehicle's thruster configuration.  
  In *emergency scenarios*, this topic is also used by the [`system_monitor_node`](#node) to send a **zero-force command** to stop the vehicle.  
  It serves as the **final step in the control pipeline** before converting force into actuation signals.
