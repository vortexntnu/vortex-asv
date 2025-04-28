# Thrust Allocator 

This package provides the implementation of the thrust allocator for the Vortex ASV.  
It computes individual thruster outputs based on a desired thrust vector using the **unweighted pseudoinverse-based allocation method**, as described in *Fossen (2011)* — *Handbook of Marine Craft Hydrodynamics and Motion Control* (Chapter 12.3.2).

## Node

The `thrust_allocator_asv_node` manages **thruster power allocation**, converting the required forces and torques into individual thruster outputs using an optimal allocation strategy.

### Goal

- Implemented as a **LifecycleNode**, which uses a state model to manage transitions between operational states:  
  > *Unconfigured → Inactive → Active → Finalized*
- **Receives motion commands** on [`/freya/wrench_input`](../../control/hybridpath_controller/README.md#freyawrench_input), representing desired forces and torques (*surge*, *sway*, *yaw*), and translates them into thrust values for each individual thruster.
- **Optimal thrust allocation** using the *pseudo-inverse* of the configuration matrix to efficiently distribute forces among thrusters.
- **Command saturation** ensures thrust values stay within operational limits and do not exceed the physical capabilities of the actuators.
- **Periodic publication** of thrust commands to [`/freya/thruster_forces`](#freyathruster_forces) using an internal timer.

### Subscribers

- [`/freya/wrench_input`](../../control/hybridpath_controller/README.md#freyawrench_input)

### Publishers

- [`/freya/thruster_forces`](#freyathruster_forces)
- [`/freya/thrust_allocator_asv_node/transition_event`](#freyathrust_allocator_asv_nodetransition_event)

## Topics

- ### /freya/thruster_forces
  
  | Topic Info         |                                  |
  |--------------------|----------------------------------|
  | **Message type**   | [`std_msgs/msg/Float64MultiArray`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float64MultiArray.html) |
  | **Published by**   | [`thrust_allocator_asv_node`](#node) <br> [`system_monitor_node`](../../mission/system_monitor/README.md#node) |
  | **Subscribed by**  | [`thruster_interface_asv_node`](../../mission/joystick_interface_asv/README.md#node) |

  #### Function  
  Transmits the **individual force values** to be applied to each thruster of the ASV.  
  These forces are calculated by the [`thrust_allocator_asv_node`](#node) based on the desired motion commands and the vehicle's thruster configuration.  
  In *emergency scenarios*, this topic is also used by the [`system_monitor_node`](../../mission/system_monitor/README.md#node) to send a **zero-force command** to stop the vehicle.  
  It serves as the **final step in the control pipeline** before converting force into actuation signals.

- ### `/freya/thrust_allocator_asv_node/transition_event`
  
  | Topic Info         |                                  |
  |--------------------|----------------------------------|
  | **Message type**   | [`lifecycle_msgs/msg/TransitionEvent`](https://docs.ros2.org/foxy/api/lifecycle_msgs/msg/TransitionEvent.html) |
  | **Published by**   | `thrust_allocator_asv_node` |
  | **Subscribed by**  | *None* |
  
  #### Function
  
  This topic is automatically published by ROS 2 when a node implemented as a **LifecycleNode** changes its internal state.  
  It broadcasts lifecycle state transitions of the `thrust_allocator_asv_node`.  
  The topic is primarily used by system tools or monitoring nodes to track the operational state of the thrust allocator (e.g., *unconfigured*, *inactive*, *active*, *shutting down*).  
  It supports **system transparency** and assists in **debugging** and managing the node’s lifecycle remotely.

- ### [`/freya/wrench_input`](../../control/hybridpath_controller/README.md#freyawrench_input)
