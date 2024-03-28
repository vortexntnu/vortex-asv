import numpy as np
import math
from d_star_lite.d_star_lite_node import DSLNode
from geometry_msgs.msg import Point

# Link to the original code:
# https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DStarLite/d_star_lite.py

# Link to theory:
# http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf


class DStarLite:
    """
    Implements the D* Lite algorithm for path planning in a grid.

    This class manages the pathfinding grid, obstacles and calculates the shortest path from a start DSLNode to a goal DSLNode.
    
    Methods:
    -------------------------------------------------------------------------------------------
        __init__(ox: list, oy: list, min_dist_to_obstacle: float = 4.5): Initializes a new instance of the DStarLite class.
        
        create_grid(val: float) -> np.ndarray: Creates a grid initialized with a specific value.
        
        is_obstacle(DSLNode: DSLNode) -> bool: Check if the DSLNode is considered an obstacle or is too close to an obstacle.
        
        movement_cost(node1: DSLNode, node2: DSLNode) -> float: Calculates the cost of moving from node1 to node2.
       
        heuristic_distance(s: DSLNode) -> float: Calculates the heuristic distance from DSLNode s to the goal using the Euclidean distance.
      
        calculate_key(s: DSLNode) -> tuple: Calculates the priority key for a DSLNode 's' based on the D* Lite algorithm.
      
        is_valid(DSLNode: DSLNode) -> bool: Determines if a DSLNode is within the grid boundaries.
        
        pred(u: DSLNode) -> list[DSLNode]: Retrieves the predecessors of a DSLNode 'u'.
        
        initialize(start: DSLNode, goal: DSLNode): Initializes the grid and the D* Lite algorithm.
        
        update_vertex(u: DSLNode): Updates the vertex in the priority queue and the rhs value of the DSLNode 'u'.
        
        detect_and_update_waypoints(current_point: DSLNode, next_point: DSLNode): Updates the waypoints based on the current and next points.
        
        compare_keys(key_pair1: tuple[float, float], key_pair2: tuple[float, float]) -> bool: Compares the priority keys of two nodes.
        
        compute_shortest_path(): Computes or recomputes the shortest path from the start to the goal using the D* Lite algorithm.
        
        compute_current_path() -> list[DSLNode]: Computes the current path from the start to the goal.
        
        get_WP() -> list[list[int]]: Retrieves the waypoints and adjusts their coordinates to the original coordinate system.
        
        dsl_main(start: DSLNode, goal: DSLNode) -> tuple[bool, list[int], list[int]]: Main function to run the D* Lite algorithm.
    """

    possible_motions = [ # Represents the possible motions in the grid and corresponding costs
        DSLNode(1, 0, 1), DSLNode(0, 1, 1), DSLNode(-1, 0, 1), DSLNode(0, -1, 1), 
        DSLNode(1, 1, math.sqrt(2)), DSLNode(1, -1, math.sqrt(2)),
        DSLNode(-1, 1, math.sqrt(2)), DSLNode(-1, -1, math.sqrt(2))
    ]

    def __init__(self, obstacles: list[Point], start: Point, goal: Point, min_dist_to_obstacle: float = 4.5, origin: Point = Point(x=0.0,y=0.0), height: int = 25, width: int = 25):
        """
        Initializes a new instance of the DStarLite class.

        Args:
            obstacles (list): A list of Point objects representing the obstacles.
            start (DSLNode): The start node.
            goal (DSLNode): The goal node.
            min_dist_to_obstacle (float): The minimum distance a DSLNode must be from any obstacle to be considered valid. Defaults to 4.5.
            origin (tuple): The origin of the grid. Defaults to (0, 0).
            height (int): The height of the grid. Defaults to 25.
            width (int): The width of the grid. Defaults to 25.
        """
        if len(obstacles) == 0: # If no obstacles are provided
            self.obstacles = []

        else:
            self.obstacles = [DSLNode(int(point.x), int(point.y)) for point in obstacles] # The obstacles as nodes
            self.obstacles_xy = np.array( # The obstacles as xy coordinates
                [[obstacle.x, obstacle.y] for obstacle in self.obstacles]
            )
        
        self.start = DSLNode(int(start.x), int(start.y)) # The start DSLNode
        self.goal = DSLNode(int(goal.x), int(goal.y)) # The goal DSLNode
        self.world_grid_origin = (int(origin.x), int(origin.y)) # The origin of the world grid
        self.world_grid_height = height/2 # The height of the world grid
        self.world_grid_width = width/2 # The width of the world grid

        # Compute the min and max values for the world boundaries
        self.x_min = int(origin.x-self.world_grid_width)
        self.y_min = int(origin.y-self.world_grid_height)
        self.x_max = int(origin.x+self.world_grid_width)
        self.y_max = int(origin.y+self.world_grid_height)

        self.priority_queue = [] # Priority queue
        self.key_min = 0.0 # The minimum key in priority queue
        self.key_old = 0.0 # The old minimum key in priority queue
        self.rhs = self.create_grid(float("inf")) # The right hand side values
        self.g = self.create_grid(float("inf")) # The g values
        self.initialized = False # Whether the grid has been initialized
        self.waypoints = [] # The waypoints
        self.min_dist_to_obstacle = min_dist_to_obstacle # The minimum distance a DSLNode must be from any obstacle to be considered valid

    def create_grid(self, val: float) -> np.ndarray:
        """
        Creates a grid initialized with a specific value.

        Args:
            val (float): The value to initialize the grid with.

        Returns:
            np.ndarray: A 2D numpy array representing the initialized grid.
        """
        return np.full((self.x_max - self.x_min, self.y_max - self.y_min), val)
    
    def is_obstacle(self, dslnode: DSLNode) -> bool:
        """
        Check if the DSLNode is considered an obstacle or is too close to an obstacle.

        Args:
            DSLNode (DSLNode): The DSLNode to check.

        Returns:
            bool: True if the DSLNode is too close to an obstacle or is an obstacle, False otherwise.
        """
        # If there are no obstacles, return False
        if len(self.obstacles) == 0:
            return False
        
        # Convert the DSLNode's coordinates to a numpy array for efficient distance computation
        node_xy = np.array([dslnode.x, dslnode.y])

        # Compute the euclidean distances from the DSLNode to all obstacles
        distances = np.sqrt(np.sum((self.obstacles_xy - node_xy) ** 2, axis=1))

        # Check if any distance is less than the minimum distance (default: 4.5)
        return np.any(distances < self.min_dist_to_obstacle)
    
    def movement_cost(self, node1: DSLNode, node2: DSLNode) -> float:
        """
        Calculates the cost of moving from node1 to node2. Returns infinity if node2 is an obstacle or if no valid motion is found.

        Args:
            node1 (DSLNode): The starting DSLNode.
            node2 (DSLNode): The ending DSLNode.

        Returns:
            float: The cost of moving from node1 to node2.
        """
        if self.is_obstacle(node2):
            return math.inf
        
        movement_vector = DSLNode(node1.x - node2.x, node1.y - node2.y)
        for motion in self.possible_motions:
            if motion == movement_vector:
                return motion.cost
        return math.inf
    
    def heuristic_distance(self, s: DSLNode) -> float:
        """
        Calculates the heuristic distance from DSLNode s to the goal using the Euclidean distance.

        Args:
            s (DSLNode): The DSLNode to calculate the heuristic distance from.

        Returns:
            float: The heuristic distance from DSLNode s to the goal.
        """
        return DSLNode.distance_between_nodes(s, self.goal) # Euclidean distance
    
    def calculate_key(self, s: DSLNode) -> tuple:
        """
        Calculates the priority key for a DSLNode 's' based on the D* Lite algorithm.

        The key is a tuple consisting of two parts:
        1. The estimated total cost from the start to the goal through 's', combining the minimum of g(s) and rhs(s),
          the heuristic distance to the goal and a constant key_min.
        2. The minimum of g(s) and rhs(s) representing the best known cost to reach 's'.

        Args:
            s (DSLNode): The DSLNode to calculate the key for.

        Returns:
            tuple: A tuple of two floats representing the priority key for the DSLNode.
        """
        return (min(self.g[s.x][s.y], self.rhs[s.x][s.y]) + self.heuristic_distance(s) + self.key_min, 
                min(self.g[s.x][s.y], self.rhs[s.x][s.y]))
    
    def is_valid(self, DSLNode: DSLNode) -> bool:
        """
        Determines if a DSLNode is within the grid boundaries.

        Args:
            DSLNode (DSLNode): The DSLNode to check.
        
        Returns:
            bool: True if the DSLNode is within the grid boundaries, False otherwise.
        """
        return self.x_min <= DSLNode.x < self.x_max and self.y_min <= DSLNode.y < self.y_max
    
    def pred(self, u: DSLNode) -> list[DSLNode]:
        """
        Retrieves the predecessors of a DSLNode 'u'. In this case, the predecessors are the valid neighbours of the DSLNode.

        Args:
            u (DSLNode): The DSLNode to retrieve predecessors for.
        
        Returns:
            list: A list of predecessors of the DSLNode 'u'.
        """
        return [u + motion for motion in self.possible_motions if self.is_valid(u + motion)]
    
    def initialize(self):
        """
        Initializes the grid and the D* Lite algorithm.
        This function adjusts the coordinates of the start and goal nodes based on the grid's minimum world coordinates,
        sets up the cost and right-hand side (rhs) grids, and initializes the priority queue. This setup is required
        before the algorithm begins pathfinding. The initialization will only occur once; subsequent calls to this
        function will have no effect.

        Args:
            start (DSLNode): The start DSLNode.
            goal (DSLNode): The goal DSLNode.
        """
        
        if not self.initialized:
            self.initialized = True
            print("Initializing")
            self.priority_queue = []
            self.key_min = 0.0
            self.rhs = self.create_grid(math.inf)
            self.g = self.create_grid(math.inf)
            self.rhs[self.goal.x][self.goal.y] = 0
            self.priority_queue.append((self.goal, self.calculate_key(self.goal)))

    def update_vertex(self, u: DSLNode):
        """
        Updates the vertex in the priority queue and the rhs value of the DSLNode 'u'.

        This method adjusts the right-hand side (rhs) value for a DSLNode unless it's the goal. It also ensures that the
        DSLNode's priority in the queue reflects its current g and rhs values, reordering the queue as necessary.

        Args:
            u (DSLNode): The DSLNode to update.

        """
        if not u == self.goal:
            self.rhs[u.x][u.y] = min([self.movement_cost(u, sprime) + self.g[sprime.x][sprime.y] for sprime in self.pred(u)])

        # Update the priority queue  
        if any([u == dslnode for dslnode, key in self.priority_queue]):
            self.priority_queue = [(dslnode, key) for dslnode, key in self.priority_queue if not u == dslnode]
            self.priority_queue.sort(key=lambda x: x[1])
        if self.g[u.x][u.y] != self.rhs[u.x][u.y]:
            self.priority_queue.append((u, self.calculate_key(u)))

        # Resort the priority queue
        self.priority_queue.sort(key=lambda x: x[1])
    
    def detect_and_update_waypoints(self, current_point: DSLNode, next_point: DSLNode):
        """
        Updates the waypoints based on the current and next points.

        This function checks the direction between the last waypoint and the current point, and the direction
        between the current point and the next point. If there is a change in direction, indicating a turn or
        deviation in the path, the current point is added to the list of waypoints.

        Args:
            current_point (DSLNode): The current point.
            next_point (DSLNode): The next point.
        """
        if not self.waypoints:  # If the waypoint list is empty
            self.waypoints.append(current_point)
        else:
            # Get the last waypoint
            last_wp = self.waypoints[-1]
            # Determine directions
            last_direction = DSLNode.get_direction(last_wp, current_point)
            current_direction = DSLNode.get_direction(current_point, next_point)
            
            # If there's a change in direction, add the current point to waypoints
            if current_direction != last_direction:
                #print("Change in direction detected")
                self.waypoints.append(current_point)

    def compare_keys(self, key_pair1: tuple[float, float], key_pair2: tuple[float, float]) -> bool:
        """
        Compares the priority keys of two nodes.

        Args:
            key_pair1 (tuple): The first key pair to compare.
            key_pair2 (tuple): The second key pair to compare.
        
        Returns:
            bool: True if `key_pair1` should precede `key_pair2` in sorting order, False otherwise.
        """
        return key_pair1[0] < key_pair2[0] or (key_pair1[0] == key_pair2[0] and key_pair1[1] < key_pair2[1])
    
    def compute_shortest_path(self):
        """
        Computes or recomputes the shortest path from the start to the goal using the D* Lite algorithm.

        This method iteratively updates the priorities and costs of nodes based on the graph's current state,
        adjusting the path as necessary until the start DSLNode's key does not precede the smallest key in the
        priority queue and the start DSLNode's rhs and g values are equal.
        """

        # Loop until the priority queue is empty, indicating no more nodes to process
        while self.priority_queue:
            # Sort the priority queue based on the keys to ensure the node with the smallest key is processed first
            self.priority_queue.sort(key=lambda x: x[1])

            # Extract the smallest key and its corresponding node
            k_old = self.priority_queue[0][1]
            u = self.priority_queue[0][0]
            
            # Double-check conditions to potentially exit the loop
            has_elements = len(self.priority_queue) > 0
            start_key = self.calculate_key(self.start)

            # Determine if the start node's key is outdated or not, affecting loop continuation
            start_key_not_updated = has_elements and self.compare_keys(self.priority_queue[0][1], start_key)

            # Check if the rhs value and g value for the start node are equal, indicating optimality reached for the start node
            rhs_not_equal_to_g = self.rhs[self.start.x][self.start.y] != self.g[self.start.x][self.start.y]
            
            # Exit the loop if no updates are required
            if not start_key_not_updated and not rhs_not_equal_to_g:
                break
            
            # Remove the processed node from the priority queue
            self.priority_queue.pop(0)

            # If the current node's old key is outdated, reinsert it with the updated key
            if self.compare_keys(k_old, self.calculate_key(u)):
                self.priority_queue.append((u, self.calculate_key(u)))

            # If the g value is greater than the rhs value, update it to achieve consistency
            elif self.g[u.x][u.y] > self.rhs[u.x][u.y]:
                self.g[u.x][u.y] = self.rhs[u.x][u.y]

                # Update all predecessors of the current node as their cost might have changed
                for s in self.pred(u):
                    self.update_vertex(s)

            # If no optimal path is known (g value is infinity), set the current node's g value to
            # infinity and update its predecessors
            else:
                self.g[u.x][u.y] = float('inf')

                # Update the current node and its predecessors
                for s in self.pred(u) + [u]:
                    self.update_vertex(s)

    def compute_current_path(self) -> list[DSLNode]:
        """
        Computes the current path from the start to the goal.

        Returns:
            list: A list of DSLNode objects representing the current path from the start to the goal.
        """
        path = list()
        current_point = DSLNode(self.start.x, self.start.y)
        last_point = None
        
        while not current_point == self.goal:
            if last_point is not None:
                self.detect_and_update_waypoints(last_point, current_point)
            path.append(current_point)
            last_point = current_point
            current_point = min(self.pred(current_point), key = lambda sprime: self.movement_cost(current_point, sprime) + self.g[sprime.x][sprime.y])
        path.append(self.goal)
        self.waypoints.append(self.goal)

        return path
    
    
    def get_WP(self) -> list[Point]:
        """
        Retrieves the waypoints and adjusts their coordinates to the original coordinate system.

        Returns:
            list: A list of waypoints with their coordinates adjusted to the original coordinate system.
        """
        WP_list = []
        for wp in self.waypoints:
            WP_list.append(Point(x=float(wp.x), y=float(wp.y), z=0.0))
        return WP_list
    
    
    def dsl_main(self) -> None:
        """
        Main function to run the D* Lite algorithm.

        Args:
            start (DSLNode): The start DSLNode.
            goal (DSLNode): The goal DSLNode.
        """
        self.initialize()
        self.compute_shortest_path()
        self.compute_current_path()
        print("Path found")

        