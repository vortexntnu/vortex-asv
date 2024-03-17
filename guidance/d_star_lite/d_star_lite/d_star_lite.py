import numpy as np
import math

# Link to the original code:
# https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DStarLite/d_star_lite.py

class DSLNode:
    """
    Represents a DSLNode in the grid.

    Attributes:
        x (int): The x-coordinate of the DSLNode.
        y (int): The y-coordinate of the DSLNode.
        cost (float): The cost of moving to the DSLNode.

    """
    def __init__(self, x: int = 0, y: int = 0, cost: float = 0.0):
        """
        Initializes a new instance of the DSLNode class.

        Args:
            x (int): The x-coordinate of the DSLNode. Defaults to 0.
            y (int): The y-coordinate of the DSLNode. Defaults to 0.
            cost (float): The cost of moving to the DSLNode. Defaults to 0.0.
        """
        self.x = x
        self.y = y
        self.cost = cost

def combine_nodes(node1: DSLNode, node2: DSLNode) -> DSLNode:
    """
    Combines two DSLNode objects by summing their x and y coordinates and their costs.

    Args:
        node1 (DSLNode): The first DSLNode to combine.
        node2 (DSLNode): The second DSLNode to combine.

    Returns:
        DSLNode: A new DSLNode object with the combined x and y coordinates and costs.
    """
    new_node = DSLNode()
    new_node.x = node1.x + node2.x
    new_node.y = node1.y + node2.y
    new_node.cost = node1.cost + node2.cost
    return new_node

def compare_coordinates(node1: DSLNode, node2: DSLNode) -> bool:
    """
    Checks if two DSLNode objects have the same x and y coordinates.

    Args:
        node1 (DSLNode): The first DSLNode to compare.
        node2 (DSLNode): The second DSLNode to compare.

    Returns:
        bool: True if the nodes have the same x and y coordinates, False otherwise.
    """
    return node1.x == node2.x and node1.y == node2.y

def distance(node1: DSLNode, node2: DSLNode) -> float:
    """
    Computes the Euclidean distance between two DSLNode objects.

    Args:
        node1 (DSLNode): The first DSLNode.
        node2 (DSLNode): The second DSLNode.
    
    Returns:
        float: The Euclidean distance between the two nodes.
    """
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

class DStarLite:
    """
    Implements the D* Lite algorithm for path planning in a grid.

    This class manages the pathfinding grid, obstacles and calculates the shortest path from a start DSLNode to a goal DSLNode.
    
    Methods:
    -------------------------------------------------------------------------------------------
        __init__(ox: list, oy: list, dist_to_obstacle: float = 4.5): Initializes a new instance of the DStarLite class.
        
        create_grid(val: float) -> np.ndarray: Creates a grid initialized with a specific value.
        
        is_obstacle(DSLNode: DSLNode) -> bool: Check if the DSLNode is considered an obstacle or is too close to an obstacle.
        
        movement_cost(node1: DSLNode, node2: DSLNode) -> float: Calculates the cost of moving from node1 to node2.
       
        heuristic_distance(s: DSLNode) -> float: Calculates the heuristic distance from DSLNode s to the goal using the Euclidean distance.
      
        calculate_key(s: DSLNode) -> tuple: Calculates the priority key for a DSLNode 's' based on the D* Lite algorithm.
      
        is_valid(DSLNode: DSLNode) -> bool: Determines if a DSLNode is within the grid boundaries.
        
        get_neighbours(u: DSLNode) -> list[DSLNode]: Generates a list of valid neighbours of a DSLNode 'u'.
        
        pred(u: DSLNode) -> list[DSLNode]: Retrieves the predecessors of a DSLNode 'u'.
        
        initialize(start: DSLNode, goal: DSLNode): Initializes the grid and the D* Lite algorithm.
        
        update_vertex(u: DSLNode): Updates the vertex in the priority queue and the rhs value of the DSLNode 'u'.
        
        get_direction(node1: DSLNode, node2: DSLNode) -> tuple: Calculates the direction from node1 to node2.
        
        detect_and_update_waypoints(current_point: DSLNode, next_point: DSLNode): Updates the waypoints based on the current and next points.
        
        compare_keys(key_pair1: tuple[float, float], key_pair2: tuple[float, float]) -> bool: Compares the priority keys of two nodes.
        
        compute_shortest_path(): Computes or recomputes the shortest path from the start to the goal using the D* Lite algorithm.
        
        compute_current_path() -> list[DSLNode]: Computes the current path from the start to the goal.
        
        get_WP() -> list[list[int]]: Retrieves the waypoints and adjusts their coordinates to the original coordinate system.
        
        dsl_main(start: DSLNode, goal: DSLNode) -> tuple[bool, list[int], list[int]]: Main function to run the D* Lite algorithm.
    """

    motions = [
        DSLNode(1, 0, 1), DSLNode(0, 1, 1), DSLNode(-1, 0, 1), DSLNode(0, -1, 1), 
        DSLNode(1, 1, math.sqrt(2)), DSLNode(1, -1, math.sqrt(2)),
        DSLNode(-1, 1, math.sqrt(2)), DSLNode(-1, -1, math.sqrt(2))
    ]

    def __init__(self, ox: list, oy: list, dist_to_obstacle: float = 4.5):
        """
        Initializes a new instance of the DStarLite class.

        Args:
            ox (list): The x-coordinates of the obstacles.
            oy (list): The y-coordinates of the obstacles.
            dist_to_obstacle (float): The minimum distance a DSLNode must be from any obstacle to be considered valid. Defaults to 4.5.
        """
        self.x_min_world = int(min(ox)) # The minimum x and y coordinates of the grid
        self.y_min_world = int(min(oy)) # The minimum x and y coordinates of the grid
        self.x_max = int(abs(max(ox) - self.x_min_world)) # The maximum x and y coordinates of the grid
        self.y_max = int(abs(max(oy) - self.y_min_world)) # The maximum x and y coordinates of the grid
        self.obstacles = [DSLNode(x - self.x_min_world, y - self.y_min_world) for x, y in zip(ox, oy)] # The obstacles
        self.obstacles_xy = np.array( # Numpy array for of obstacle coordinates
            [[obstacle.x, obstacle.y] for obstacle in self.obstacles]
        )
        self.start = DSLNode(0, 0) # The start DSLNode
        self.goal = DSLNode(0, 0) # The goal DSLNode
        self.U = [] # Priority queue
        self.km = 0.0 # The minimum key in U
        self.kold = 0.0 # The old minimum key in U
        self.rhs = self.create_grid(float("inf")) # The right hand side values
        self.g = self.create_grid(float("inf")) # The g values
        self.initialized = False # Whether the grid has been initialized
        self.WP = [] # The waypoints
        self.dist_to_obstacle = dist_to_obstacle # The minimum distance a DSLNode must be from any obstacle to be considered valid

    def create_grid(self, val: float) -> np.ndarray:
        """
        Creates a grid initialized with a specific value.

        Args:
            val (float): The value to initialize the grid with.

        Returns:
            np.ndarray: A 2D numpy array representing the initialized grid.
        """
        return np.full((self.x_max, self.y_max), val)
    
    def is_obstacle(self, DSLNode: DSLNode) -> bool:
        """
        Check if the DSLNode is considered an obstacle or is too close to an obstacle.

        Args:
            DSLNode (DSLNode): The DSLNode to check.

        Returns:
            bool: True if the DSLNode is too close to an obstacle or is an obstacle, False otherwise.
        """
        # Convert the DSLNode's coordinates to a numpy array for efficient distance computation
        node_xy = np.array([DSLNode.x, DSLNode.y])

        # Compute the euclidean distances from the DSLNode to all obstacles
        distances = np.sqrt(np.sum((self.obstacles_xy - node_xy) ** 2, axis=1))

        # Check if any distance is less than the minimum distance (default: 4.5)
        return np.any(distances < self.dist_to_obstacle)
    
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
        for motion in self.motions:
            if compare_coordinates(motion, movement_vector):
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
        return distance(s, self.goal) # Euclidean distance
    
    def calculate_key(self, s: DSLNode) -> tuple:
        """
        Calculates the priority key for a DSLNode 's' based on the D* Lite algorithm.

        The key is a tuple consisting of two parts:
        1. The estimated total cost from the start to the goal through 's', combining the minimum of g(s) and rhs(s),
          the heuristic distance to the goal and a constant km.
        2. The minimum of g(s) and rhs(s) representing the best known cost to reach 's'.

        Args:
            s (DSLNode): The DSLNode to calculate the key for.

        Returns:
            tuple: A tuple of two floats representing the priority key for the DSLNode.
        """
        return (min(self.g[s.x][s.y], self.rhs[s.x][s.y]) + self.heuristic_distance(s) + self.km, 
                min(self.g[s.x][s.y], self.rhs[s.x][s.y]))
    
    def is_valid(self, DSLNode: DSLNode) -> bool:
        """
        Determines if a DSLNode is within the grid boundaries.

        Args:
            DSLNode (DSLNode): The DSLNode to check.
        
        Returns:
            bool: True if the DSLNode is within the grid boundaries, False otherwise.
        """
        return 0 <= DSLNode.x < self.x_max and 0 <= DSLNode.y < self.y_max
    
    def get_neighbours(self, u: DSLNode) -> list[DSLNode]:
        """
        Generates a list of valid neighbours of a DSLNode 'u'.

        Args:
            u (DSLNode): The DSLNode to generate neighbours for.

        Returns:
            list: A list of valid neighbours of the DSLNode 'u'.
        """
        return [combine_nodes(u, motion) for motion in self.motions if self.is_valid(combine_nodes(u, motion))]
    
    def pred(self, u: DSLNode) -> list[DSLNode]:
        """
        Retrieves the predecessors of a DSLNode 'u'. In this case, the predecessors are the neighbours of the DSLNode.

        Args:
            u (DSLNode): The DSLNode to retrieve predecessors for.
        
        Returns:
            list: A list of predecessors of the DSLNode 'u'.
        """
        return self.get_neighbours(u)
    
    def initialize(self, start: DSLNode, goal: DSLNode):
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
        self.start.x = start.x - self.x_min_world
        self.start.y = start.y - self.y_min_world
        self.goal.x = goal.x - self.x_min_world
        self.goal.y = goal.y - self.y_min_world
        if not self.initialized:
            self.initialized = True
            print("Initializing")
            self.U = []
            self.km = 0.0
            self.rhs = self.create_grid(math.inf)
            self.g = self.create_grid(math.inf)
            self.rhs[self.goal.x][self.goal.y] = 0
            self.U.append((self.goal, self.calculate_key(self.goal)))

    def update_vertex(self, u: DSLNode):
        """
        Updates the vertex in the priority queue and the rhs value of the DSLNode 'u'.

        This method adjusts the right-hand side (rhs) value for a DSLNode unless it's the goal. It also ensures that the
        DSLNode's priority in the queue reflects its current g and rhs values, reordering the queue as necessary.

        Args:
            u (DSLNode): The DSLNode to update.

        """
        if not compare_coordinates(u, self.goal):
            self.rhs[u.x][u.y] = min([self.movement_cost(u, sprime) + self.g[sprime.x][sprime.y] for sprime in self.pred(u)])

        # Update the priority queue    
        if any([compare_coordinates(u, DSLNode) for DSLNode, key in self.U]):
            self.U = [(DSLNode, key) for DSLNode, key in self.U if not compare_coordinates(DSLNode, u)]
            self.U.sort(key=lambda x: x[1])
        if self.g[u.x][u.y] != self.rhs[u.x][u.y]:
            self.U.append((u, self.calculate_key(u)))

        # Resort the priority queue
        self.U.sort(key=lambda x: x[1])

    def get_direction(self, node1: DSLNode, node2: DSLNode) -> tuple:
        """
        Calculates the direction from node1 to node2.

        Args:
            node1 (DSLNode): The starting DSLNode.
            node2 (DSLNode): The ending DSLNode.
        
        Returns:
            tuple: A tuple of two integers representing the direction from node1 to node2.
        """
        dx = node2.x - node1.x
        dx = dx/abs(dx) if dx != 0 else 0
        dy = node2.y - node1.y
        dy = dy/abs(dy) if dy != 0 else 0
        return dx, dy
    
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
        if not self.WP:  # If the waypoint list is empty
            self.WP.append(current_point)
        else:
            # Get the last waypoint
            last_wp = self.WP[-1]
            # Determine directions
            last_direction = self.get_direction(last_wp, current_point)
            current_direction = self.get_direction(current_point, next_point)
            
            # If there's a change in direction, add the current point to waypoints
            if current_direction != last_direction:
                #print("Change in direction detected")
                self.WP.append(current_point)

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
        self.U.sort(key=lambda x: x[1])
        has_elements = len(self.U) > 0
        start_key_not_updated = self.compare_keys(self.U[0][1], self.calculate_key(self.start))
        rhs_not_equal_to_g = self.rhs[self.start.x][self.start.y] != self.g[self.start.x][self.start.y]
        while has_elements and start_key_not_updated or rhs_not_equal_to_g:
            self.kold = self.U[0][1]
            u = self.U[0][0]
            self.U.pop(0)
            if self.compare_keys(self.kold, self.calculate_key(u)):
                self.U.append((u, self.calculate_key(u)))
                self.U.sort(key=lambda x: x[1])
            elif (self.g[u.x][u.y] > self.rhs[u.x, u.y]).any():
                self.g[u.x, u.y] = self.rhs[u.x, u.y]
                for s in self.pred(u):
                    self.update_vertex(s)
            else:
                self.g[u.x, u.y] = math.inf
                for s in self.pred(u) + [u]:
                    self.update_vertex(s)
            self.U.sort(key=lambda x: x[1])
            start_key_not_updated = self.compare_keys(self.U[0][1], self.calculate_key(self.start))
            rhs_not_equal_to_g = self.rhs[self.start.x][self.start.y] != self.g[self.start.x][self.start.y]
    
    def compute_current_path(self) -> list[DSLNode]:
        """
        Computes the current path from the start to the goal.

        Returns:
            list: A list of DSLNode objects representing the current path from the start to the goal.
        """
        path = list()
        current_point = DSLNode(self.start.x, self.start.y)
        last_point = None

        while not compare_coordinates(current_point, self.goal):
            if last_point is not None:
                self.detect_and_update_waypoints(last_point, current_point)
            path.append(current_point)
            last_point = current_point
            current_point = min(self.pred(current_point), key = lambda sprime: self.movement_cost(current_point, sprime) + self.g[sprime.x][sprime.y])
        path.append(self.goal)
        self.WP.append(self.goal)

        return path
    
    
    def get_WP(self) -> list[list[int]]:
        """
        Retrieves the waypoints and adjusts their coordinates to the original coordinate system.

        Returns:
            list: A list of waypoints with their coordinates adjusted to the original coordinate system.
        """
        WP_list = []
        for wp in self.WP:
            WP_list.append([wp.x + self.x_min_world, wp.y + self.y_min_world])
        return WP_list
    
    
    def dsl_main(self, start: DSLNode, goal: DSLNode) -> tuple[bool, list[int], list[int]]:
        """
        Main function to run the D* Lite algorithm.

        Args:
            start (DSLNode): The start DSLNode.
            goal (DSLNode): The goal DSLNode.
        
        Returns:
            tuple: A tuple containing a boolean indicating if the path was found, and the x and y coordinates of the path.  
        """ 
        pathx = []
        pathy = []
        self.initialize(start, goal)
        self.compute_shortest_path()
        pathx.append(self.start.x + self.x_min_world)
        pathy.append(self.start.y + self.y_min_world)
        print("Path found")
        return True, pathx, pathy


    
                            