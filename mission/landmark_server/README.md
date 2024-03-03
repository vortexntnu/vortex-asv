# Landmark Server
this ROS-node serves as an interface between the perception system and the control system. The node receives messages of type landmarkArray defined `vortex-msgs/msg/LandmarkArray.msg`. 
The landmarks are recieved through a topic published by a node in the `vortex-target-tracking`repo. This server then publishes the odometries of these landmarks on a topic the control system suscribes to. 
The landmark server node is configured as an action server allowing the control systems to filter what landmarks they want to recieve.

## Storing Landmarks
The landmark server receives the landmarks and stores them in the class member `storedLandmarks_`. The server handles logic for adding, updating and removing landmarks.

## Publishers
The landmark server supports publising of all the landmarks currently stored by the server over the `landmarks_out` topic.
The server also supports publishing the poses for all the currently stored landmarks over the `landmark_poses_out`. These poses are published as a PoseArray message that can be visualized in the foxglove 3D-panel.

## Action
The control systems can send action request for the FilteredLandmarks action defined in `vortex-msgs/action/FilteredLandmarks.action`. The control systems can filter landmarks by type and distance in the action request.

```yaml
float32 IGNORE_DISTANCE =0.0

# Define the action request
string[] landmark_types
float32  distance
string  frame_id
```
The server returns the odometries of landmarks that have a landmark type that matches one of the elements in `string[] landmark_types` and is withing the distance specified in the request.
The distance is the relative distance between the landmark and the drone. 
If `string[] landmark_types` is empty then it will accept all types within the specified distance.
If distance is set to `0.0` the distance is ignored and the server returns all landmarks matching the `string[] landmark_types` filter.
`string  frame_id` is used to specify which frame is to be used on the transformation to calculate the relative distance between the landmark and the drone.




