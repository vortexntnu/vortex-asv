# Thruster Interface ASV

This package provides an interface for controlling the thrusters of freya converting forces values to pwm values. Similarly as what done with orca (see https://github.com/vortexntnu/vortex-auv/tree/main/motion/thruster_interface_auv) the mapping is based on a piecewise third order polynomial approximating the datasheet .csv table found in /resources. Values send via i2c protocol. Below it's shown how to derive the coefficients from the .csv table provided in /resources. 

The analysis covers the table splitted in two halfs:
1) values > 0 + the leftmost green 0-valued point for a better fitting (see picture below)
2) values < 0 with the rightmost red 0-valued point

![fitting](resources/poly_approx_image.png)

## Usage

```sh
source install/setup.bash
ros2 launch thruster_interface_asv thruster_interface_asv.launch.py
```

## Structure

### Nodes

1. `thruster_interface_asv_node.cpp` contains the main loop of the node

2. `thruster_interface_asv_ros.cpp` contains the implementation of the node dependent on ros2 creating a subscriber for thruster forces, a publisher for pwm values, and a driver for handling the mapping. Initialize everything extracting all the parameters from .yaml file found in `../asv_setup/config/robots/freya.yaml` and `/config/thruster_interface_asv_config.yaml`.

3. `thruster_interface_asv_driver.cpp` contains the pure .cpp implementation for the mapping, conversion for i2c data format, and sending those values.

### Topics

- *subscribe to:* `/freya/thruster_forces  [std_msgs/msg/float64_multi_array]` -- array of forces to apply to each thruster.

- *publish:* `/freya/pwm_output  [std_msgs/msg/Int16MultiArray]` -- pwm command value to apply that force.

## Config

1. Edit `../asv_setup/config/robots/freya.yaml` for thruster parameters like mapping, direction, pwm_min and max.
2. Edit `/config/thruster_interface_asv_config.yaml` for i2c bus and address, and polynomial coefficients for the mapping.

## Contact

For questions or support, please contact [albertomorselli00@gmail.com](mailto:albertomorselli00@gmail.com).