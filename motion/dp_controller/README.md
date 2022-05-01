## ASV note
This is a tried and tested node, but is made for full 6DOF control. We will not need to use quaternions, or care about anything other than DOF 1,2 and 6. We can either rewrite this code, or use it as a reference to write a new DP controller entirely.

## DP controller

#### Subscribes to:
* /odometry/filtered

#### Publishes to:
* /auv/thruster_manager/input
* controller/mode
* debug/controlstates

#### Servers:
* Dynamic reconfigure
* Move base action server
* Control mode service server


This package provices an implementation of _Fjellstad & Fossen 1994: Quaternion Feedback Regulation of Underwater Vehicles_,
a nonlinear PD position and orientation controller, which in this implementation has been expanded with integral effect.
It provides the AUV with the ability to hold fixed setpoints in pose, heading, or a combination of these.

The core of the technical implementation lies in quaternion_pd_controller. controller_ros implements the interface layer between
the controller implementation with the rest of the ROS-based system.

The following control modes are available, which can be set using convenience functions in dp_client in the finite_state_machine package:
* Open loop         
* Pose hold         
* Heading hold   
* Pose+Heading hold

Always set control mode to open loop before switching to a different controller, such as LOS

The controller parameters can be changed during runtime because of the dynamic reconfigure server that is present
in the controller_ros file. The parameters can be und under /cfg/Controller.cfg. To reconfigure the parameters, use
```
rosrun rqt_reconfigure rqt_reconfigure
```
and select the /dp node from the list of reconfigurable nodes.
