# System monitor
This package contains a monitoring node for detecting network-related system failures on the ASV

## Node
The `system_monitor_node` implements a system-level safety monitor that checks network connectivity via IP pinging and triggers an emergency shutdown when devices become unreachable.

### Goal
- **Periodically checks** the network connectivity of critical onboard devices via IP address pinging.
- **Detects system-level failures** by monitoring the responsiveness of all listed IPs.
- **Triggers an emergency shutdown** if any device becomes unreachable:
  - Sends a lifecycle transition request to deactivate the [`thrust_allocator_asv_node`](../../motion/thrust_allocator_asv/README.md#node)
  - Publishes a zero-force command to [`/freya/thruster_forces`](#freyathruster_forces)
  - Terminates its execution, requiring **manual intervention** for recovery

### Subscribers
- *None*

### Publishers
- [`/freya/thruster_forces`](#freyathruster_forces)

## Topics
- ### [`/freya/thruster_forces`](../../motion/thrust_allocator_asv/README.md#freyathruster_forces)
