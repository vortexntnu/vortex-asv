# Dynamic Positioning (DP) Guidance

This package provides the implementation of DP guidance for the Vortex ASV.

## Usage

To use the DP guidance launch it using: `ros2 launch dp_guidance dp_guidance.launch`

To run with custom waypoints (replace example waypoints with actual waypoints, and add as many prefered):

`ros2 service call waypoint_list vortex_msgs/srv/Waypoint "waypoint: [{x: 0.0, y: 0.0, z: 0.0}, {x: 5.0, y: 5.0, z: 0.0}]"`

## Configuration

You can configure the behavior of the hybrid path guidance by modifying the parameters in the `config` directory. 