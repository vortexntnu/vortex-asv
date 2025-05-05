# ASV setup

This package contains global configs and the main launchfiles. The reason for putting these in a separate package is to be able to reach the aforementioned files from anywhere in a simple manner.

## Config
The config folder contains physical parameters related to the AUV and the environment

There are currently no physical parameters

## Launch
This package contains a launchfile for each specific ASV. Additionally the shoreside.launch.py file is to
be used on the topside computer that the joystick is connected to, for ROV operations.

### External Nodes

#### `joystick_driver`

This node is a **standard ROS 2 component** ([joy_node](https://index.ros.org/p/joy/#humble-overview)) renamed locally to `joystick_driver`.
It acts as the **interface between the physical joystick** (e.g., Xbox controller) and the ASV’s ROS 2 system, enabling **manual control** through higher-level nodes such as [`joystick_interface_asv`](../mission/joystick_interface_asv/README.md).
It reads input from the joystick hardware and **publishes state information** (axes and button values) via standard ROS 2 topics.

##### Subscribers

- `/joy/set_feedback` *(not used in this project)*

##### Publishers

- [`joy`](#joy)

---

#### `joy`

|  Topic Info       |                                |
|-------------------|--------------------------------|
| **Message type**  | [`sensor_msgs/msg/Joy`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Joy.html) |
| **Published by**  | `joystick_driver` |
| **Subscribed by** | [`joystick_interface_asv`](../mission/joystick_interface_asv/README.md#node) |

##### Function

This topic provides **raw input data** from the physical joystick, including *button states* and *axis positions*.
It serves as the **main interface for manual user input**, allowing an operator to control the ASV in real time.
The [`joystick_interface_asv`](../mission/joystick_interface_asv/README.md#node) node interprets this data to generate motion commands during *manual operation mode*.
