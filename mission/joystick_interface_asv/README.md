# Joystick Interface

This package provides a joystick-based manual control interface for the Vortex ASV, enabling direct operator interaction via Xbox controller.

## Node

The `joystick_interface_asv` node **interprets joystick input** and publishes control commands for manual operation, including *mode selection* and *direct thruster control*.  
It also handles **emergency stop** through the *killswitch*.

### Goal

- Acquires and interprets joystick input (topic [`/freya/joy`](#freyajoy) with [`Joy`](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html) messages), automatically distinguishing between *wired* and *wireless* Xbox controllers.
- Translates analogue joystick commands into force and torque vectors, applying:
    - **Configurable scaling factors** for *surge* (forward/backward), *sway* (sideways), and *yaw* (rotation)
    - **Power modulation** via LT/RT triggers acting as speed controllers
    - **Timed debounce** on buttons to prevent accidental activations
- Posts movement commands to the [`/freya/wrench_input`](../../control/hybridpath_controller/README.md#freyawrench_input) topic ([`Wrench`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Wrench.html) messages) for the thruster allocation system.
- It handles three operational states, reporting changes on the dedicated topic:
    - `XBOX_MODE`: Direct manual control via joystick (activated with button A)
    - `AUTONOMOUS_MODE`: Autonomous system delegated control (activated with button X)
    - `KILLSWITCH`: Safety state that immediately stops the vehicle by sending a zero force command (activated with button B)
- Reports the killswitch state on a dedicated topic, allowing other nodes to react appropriately to emergencies.

### Subscribers

- [`/freya/joy`](#freyajoy)

### Publishers

- [`/freya/killswitch`](#freyakillswitch)
- [`/freya/operation_mode`](#freyaoperationmode)
- [`/freya/wrench_input`](../../control/hybridpath_controller/README.md#freyawrench_input)

## Topics

- ### `/freya/joy`
  
  |  Topic Info       |                                |
  |-------------------|--------------------------------|
  | **Message type**  | [`sensor_msgs/msg/Joy`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Joy.html) |
  | **Published by**  | [`joystick_driver`](...) |
  | **Subscribed by** | `joystick_interface_asv` |
  
  #### Function
  
  This topic provides **raw input data** from the physical joystick, including *button states* and *axis positions*.  
  It serves as the **main interface for manual user input**, allowing an operator to control the ASV in real time.  
  The `joystick_interface_asv` node interprets this data to generate motion commands during *manual operation mode*.

- ### `/freya/killswitch`
  
  | Topic Info        |                                |
  |-------------------|--------------------------------|
  | **Message type**  | [`std_msgs/msg/Bool`](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html) |
  | **Published by**  | `joystick_interface_asv` |
  | **Subscribed by** | [`hybridpath_controller_node`](../../control/hybridpath_controller/README.md#node) |
  
  #### Function
  
  This topic indicates whether the **emergency stop mechanism** (*killswitch*) has been activated.
  When set to `true`, it signals the [`hybridpath_controller_node`](../../control/hybridpath_controller/README.md#node) to immediately halt operations and stop generating motion commands.  
  It serves as a **central safety mechanism** to ensure the ASV can be quickly and safely stopped in case of *manual override* or *critical failure*.

- ### `/freya/operation_mode`
  
  | Topic Info         |                                  |
  |--------------------|----------------------------------|
  | **Message type**   | [`std_msgs/msg/String`](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html) |
  | **Published by**   | `joystick_interface_asv` |
  | **Subscribed by**  | [`hybridpath_controller_node`](../../control/hybridpath_controller/README.md#node) |
  
  #### Function
  
  This topic defines the **current operating mode** of the ASV, such as `"XBOX"` for manual control or `"AUTONOMOUS"` for mission-based navigation.  
  It informs the [`hybridpath_controller_node`](../../control/hybridpath_controller/README.md#node) whether it is allowed to actively compute and publish motion commands.  
  This enables **dynamic switching** between *manual* and *autonomous* operation within the system.
