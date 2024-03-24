# D* lite
Given a map of obstacles and a minimum safe distance to the obstacles, this algorithm will compute the shortest path from the start point to the end point. This is an example of an obstacle map:

![Obstacle Map](https://drive.google.com/uc?export=download&id=1MohnPBQMoQHHbLaDkbUe43MjBPp5sTiF)

And when using this map when running the D* lite algorithm with safe distance 4.5 we get this path:

![Path](https://drive.google.com/uc?export=download&id=1il-i2aJM3pkQacTO8Jg77_2zDVcZF1ty)

A D* lite object consists of these parameters:

'''
dsl_object = DStarLite(obstacle_x, obstacle_y, start, goa, dist_to_obstacle, origin, height, width)
'''

where 'obstacle_x' and 'obstacle_y' are lists containg the x and y coordinates of the obstacles. 'start' and 'goal' are the start and goal node for the selected mission. 'origin', 'height' and 'widht' are parameters to create the world boundaries and are used to compute 'x_min', 'x_max', 'y_min' and 'y_max'. See the figures below for visual representation.

![World Grid](https://drive.google.com/uc?export=download&id=1aoxujTgjgO8oaP2H6xIE0JHGb6VVNcRs)

![Obstacle](https://drive.google.com/uc?export=download&id=1M43ohD3wpKwmgkjJ44Ut1uOT3f5MlSQI)

# D* lite ROS2 node
The node is responsible for gathering the waypoints found from the algorithm and send them to the waypoint manager. The node receives the mission parameters from the mission planner. It is both a client and a server.

