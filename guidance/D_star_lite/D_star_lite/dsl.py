import numpy as np
import math

# Link to the original code:
# https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DStarLite/d_star_lite.py

class Node:
    """
    Represents a node in the grid.

    Attributes:
        x (int): The x-coordinate of the node.
        y (int): The y-coordinate of the node.
        cost (float): The cost of moving to the node.

    """
    def __init__(self, x: int = 0, y: int = 0, cost: float = 0.0):
        """
        Initializes a new instance of the Node class.

        Args:
            x (int): The x-coordinate of the node. Defaults to 0.
            y (int): The y-coordinate of the node. Defaults to 0.
            cost (float): The cost of moving to the node. Defaults to 0.0.
        """
        self.x = x
        self.y = y
        self.cost = cost

def combine_nodes(node1: Node, node2: Node) -> Node:
    """
    Combines two Node objects by summing their x and y coordinates and their costs.

    Args:
        node1 (Node): The first node to combine.
        node2 (Node): The second node to combine.

    Returns:
        Node: A new Node object with the combined x and y coordinates and costs.
    """
    new_node = Node()
    new_node.x = node1.x + node2.x
    new_node.y = node1.y + node2.y
    new_node.cost = node1.cost + node2.cost
    return new_node

def compare_coordinates(node1: Node, node2: Node) -> bool:
    """
    Checks if two Node objects have the same x and y coordinates.

    Args:
        node1 (Node): The first node to compare.
        node2 (Node): The second node to compare.

    Returns:
        bool: True if the nodes have the same x and y coordinates, False otherwise.
    """
    return node1.x == node2.x and node1.y == node2.y

def distance(node1: Node, node2: Node) -> float:
    """
    Computes the Euclidean distance between two Node objects.

    Args:
        node1 (Node): The first node.
        node2 (Node): The second node.
    
    Returns:
        float: The Euclidean distance between the two nodes.
    """
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

class DStarLite:
    """
    Implements the D* Lite algorithm for path planning in a grid.

    This class manages the pathfinding grid, obstacles and calculates the shortest path from a start node to a goal node.

    Methods:
        __init__(ox: list, oy: list, dist_to_obstacle: float = 4.5): Initializes a new instance of the DStarLite class.

        create_grid(val: float) -> np.ndarray: Creates a grid initialized with a specific value.

        is_obstacle(node: Node) -> bool: Check if the node is considered an obstacle or is too close to an obstacle.

        movement_cost(node1: Node, node2: Node) -> float: Calculates the cost of moving from node1 to node2.

        heuristic_distance(s: Node) -> float: Calculates the heuristic distance from node s to the goal using the Euclidean distance.

        calculate_key(s: Node) -> tuple: Calculates the priority key for a node 's' based on the D* Lite algorithm.

        is_valid(node: Node) -> bool: Determines if a node is within the grid boundaries.

        get_neighbours(u: Node) -> list[Node]: Generates a list of valid neighbours of a node 'u'.

        pred(u: Node) -> list[Node]: Retrieves the predecessors of a node 'u'.

        initialize(start: Node, goal: Node): Initializes the grid and the D* Lite algorithm.

        update_vertex(u: Node): Updates the vertex in the priority queue and the rhs value of the node 'u'.

        get_direction(node1: Node, node2: Node) -> tuple: Calculates the direction from node1 to node2.

        detect_and_update_waypoints(current_point: Node, next_point: Node): Updates the waypoints based on the current and next points.

        compare_keys(key_pair1: tuple[float, float], key_pair2: tuple[float, float]) -> bool: Compares the priority keys of two nodes.

        compute_shortest_path(): Computes or recomputes the shortest path from the start to the goal using the D* Lite algorithm.

        compute_current_path() -> list[Node]: Computes the current path from the start to the goal.

        get_WP() -> list[list[int]]: Retrieves the waypoints and adjusts their coordinates to the original coordinate system.

        dsl_main(start: Node, goal: Node) -> tuple[bool, list[int], list[int]]: Main function to run the D* Lite algorithm.
    """

    motions = [
        Node(1, 0, 1), Node(0, 1, 1), Node(-1, 0, 1), Node(0, -1, 1), 
        Node(1, 1, math.sqrt(2)), Node(1, -1, math.sqrt(2)),
        Node(-1, 1, math.sqrt(2)), Node(-1, -1, math.sqrt(2))
    ]

    def __init__(self, ox: list, oy: list, dist_to_obstacle: float = 4.5):
        """
        Initializes a new instance of the DStarLite class.

        Args:
            ox (list): The x-coordinates of the obstacles.
            oy (list): The y-coordinates of the obstacles.
            dist_to_obstacle (float): The minimum distance a node must be from any obstacle to be considered valid. Defaults to 4.5.
        """
        self.x_min_world = int(min(ox)) # The minimum x and y coordinates of the grid
        self.y_min_world = int(min(oy)) # The minimum x and y coordinates of the grid
        self.x_max = int(abs(max(ox) - self.x_min_world)) # The maximum x and y coordinates of the grid
        self.y_max = int(abs(max(oy) - self.y_min_world)) # The maximum x and y coordinates of the grid
        self.obstacles = [Node(x - self.x_min_world, y - self.y_min_world) for x, y in zip(ox, oy)] # The obstacles
        self.obstacles_xy = np.array( # Numpy array for of obstacle coordinates
            [[obstacle.x, obstacle.y] for obstacle in self.obstacles]
        )
        self.start = Node(0, 0) # The start node
        self.goal = Node(0, 0) # The goal node
        self.U = [] # Priority queue
        self.km = 0.0 # The minimum key in U
        self.kold = 0.0 # The old minimum key in U
        self.rhs = self.create_grid(float("inf")) # The right hand side values
        self.g = self.create_grid(float("inf")) # The g values
        self.initialized = False # Whether the grid has been initialized
        self.WP = [] # The waypoints
        self.dist_to_obstacle = dist_to_obstacle # The minimum distance a node must be from any obstacle to be considered valid

    def create_grid(self, val: float) -> np.ndarray:
        """
        Creates a grid initialized with a specific value.

        Args:
            val (float): The value to initialize the grid with.

        Returns:
            np.ndarray: A 2D numpy array representing the initialized grid.
        """
        return np.full((self.x_max, self.y_max), val)
    
    def is_obstacle(self, node: Node) -> bool:
        """
        Check if the node is considered an obstacle or is too close to an obstacle.

        Args:
            node (Node): The node to check.

        Returns:
            bool: True if the node is too close to an obstacle or is an obstacle, False otherwise.
        """
        # Convert the node's coordinates to a numpy array for efficient distance computation
        node_xy = np.array([node.x, node.y])

        # Compute the euclidean distances from the node to all obstacles
        distances = np.sqrt(np.sum((self.obstacles_xy - node_xy) ** 2, axis=1))

        # Check if any distance is less than the minimum distance
        return np.any(distances < self.dist_to_obstacle)
    
    def movement_cost(self, node1: Node, node2: Node) -> float:
        """
        Calculates the cost of moving from node1 to node2. Returns infinity if node2 is an obstacle or if no valid motion is found.

        Args:
            node1 (Node): The starting node.
            node2 (Node): The ending node.

        Returns:
            float: The cost of moving from node1 to node2.
        """
        if self.is_obstacle(node2):
            return math.inf
        
        movement_vector = Node(node1.x - node2.x, node1.y - node2.y)
        for motion in self.motions:
            if compare_coordinates(motion, movement_vector):
                return motion.cost
        return math.inf
    
    def heuristic_distance(self, s: Node) -> float:
        """
        Calculates the heuristic distance from node s to the goal using the Euclidean distance.

        Args:
            s (Node): The node to calculate the heuristic distance from.

        Returns:
            float: The heuristic distance from node s to the goal.
        """
        return distance(s, self.goal) # Euclidean distance
    
    def calculate_key(self, s: Node) -> tuple:
        """
        Calculates the priority key for a node 's' based on the D* Lite algorithm.

        The key is a tuple consisting of two parts:
        1. The estimated total cost from the start to the goal through 's', combining the minimum of g(s) and rhs(s),
          the heuristic distance to the goal and a constant km.
        2. The minimum of g(s) and rhs(s) representing the best known cost to reach 's'.

        Args:
            s (Node): The node to calculate the key for.

        Returns:
            tuple: A tuple of two floats representing the priority key for the node.
        """
        return (min(self.g[s.x][s.y], self.rhs[s.x][s.y]) + self.heuristic_distance(s) + self.km, 
                min(self.g[s.x][s.y], self.rhs[s.x][s.y]))
    
    def is_valid(self, node: Node) -> bool:
        """
        Determines if a node is within the grid boundaries.

        Args:
            node (Node): The node to check.
        
        Returns:
            bool: True if the node is within the grid boundaries, False otherwise.
        """
        return 0 <= node.x < self.x_max and 0 <= node.y < self.y_max
    
    def get_neighbours(self, u: Node) -> list[Node]:
        """
        Generates a list of valid neighbours of a node 'u'.

        Args:
            u (Node): The node to generate neighbours for.

        Returns:
            list: A list of valid neighbours of the node 'u'.
        """
        return [combine_nodes(u, motion) for motion in self.motions if self.is_valid(combine_nodes(u, motion))]
    
    def pred(self, u: Node) -> list[Node]:
        """
        Retrieves the predecessors of a node 'u'. In this case, the predecessors are the neighbours of the node.

        Args:
            u (Node): The node to retrieve predecessors for.
        
        Returns:
            list: A list of predecessors of the node 'u'.
        """
        return self.get_neighbours(u)
    
    def initialize(self, start: Node, goal: Node):
        """
        Initializes the grid and the D* Lite algorithm.
        This function adjusts the coordinates of the start and goal nodes based on the grid's minimum world coordinates,
        sets up the cost and right-hand side (rhs) grids, and initializes the priority queue. This setup is required
        before the algorithm begins pathfinding. The initialization will only occur once; subsequent calls to this
        function will have no effect.

        Args:
            start (Node): The start node.
            goal (Node): The goal node.
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

    def update_vertex(self, u: Node):
        """
        Updates the vertex in the priority queue and the rhs value of the node 'u'.

        This method adjusts the right-hand side (rhs) value for a node unless it's the goal. It also ensures that the
        node's priority in the queue reflects its current g and rhs values, reordering the queue as necessary.

        Args:
            u (Node): The node to update.

        """
        if not compare_coordinates(u, self.goal):
            self.rhs[u.x][u.y] = min([self.movement_cost(u, sprime) + self.g[sprime.x][sprime.y] for sprime in self.pred(u)])

        # Update the priority queue    
        if any([compare_coordinates(u, node) for node, key in self.U]):
            self.U = [(node, key) for node, key in self.U if not compare_coordinates(node, u)]
            self.U.sort(key=lambda x: x[1])
        if self.g[u.x][u.y] != self.rhs[u.x][u.y]:
            self.U.append((u, self.calculate_key(u)))

        # Resort the priority queue
        self.U.sort(key=lambda x: x[1])

    def get_direction(self, node1: Node, node2: Node) -> tuple:
        """
        Calculates the direction from node1 to node2.

        Args:
            node1 (Node): The starting node.
            node2 (Node): The ending node.
        
        Returns:
            tuple: A tuple of two integers representing the direction from node1 to node2.
        """
        dx = node2.x - node1.x
        dx = dx/abs(dx) if dx != 0 else 0
        dy = node2.y - node1.y
        dy = dy/abs(dy) if dy != 0 else 0
        return dx, dy
    
    def detect_and_update_waypoints(self, current_point: Node, next_point: Node):
        """
        Updates the waypoints based on the current and next points.

        This function checks the direction between the last waypoint and the current point, and the direction
        between the current point and the next point. If there is a change in direction, indicating a turn or
        deviation in the path, the current point is added to the list of waypoints.

        Args:
            current_point (Node): The current point.
            next_point (Node): The next point.
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
        adjusting the path as necessary until the start node's key does not precede the smallest key in the
        priority queue and the start node's rhs and g values are equal.
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
    
    def compute_current_path(self) -> list[Node]:
        """
        Computes the current path from the start to the goal.

        Returns:
            list: A list of Node objects representing the current path from the start to the goal.
        """
        path = list()
        current_point = Node(self.start.x, self.start.y)
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
    
    
    def dsl_main(self, start: Node, goal: Node) -> tuple[bool, list[int], list[int]]:
        """
        Main function to run the D* Lite algorithm.

        Args:
            start (Node): The start node.
            goal (Node): The goal node.
        
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


    
                            