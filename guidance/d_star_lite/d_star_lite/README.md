# D* lite
Given a map of obstacles and a minimum safe distance to the obstacles, this algorithm will compute the shortest path from the start point to the end point. This is an example of an obstacle map:

![Obstacle Map](https://drive.google.com/uc?export=download&id=1MohnPBQMoQHHbLaDkbUe43MjBPp5sTiF)

And when using this map when running the D* lite algorithm with safe distance 4.5 we get this path:

![Path](https://drive.google.com/uc?export=download&id=1il-i2aJM3pkQacTO8Jg77_2zDVcZF1ty)

# D* lite node
The node is responsible for communicating the waypoints found from the algorithm and send them to the waypoint manager. The node receives the obstacle map from the mission planner.