import numpy as np
import random
import matplotlib.pyplot as plt
import math
from path_utils import HybridPathGenerator, HybridPathSignals

class Node:
    def __init__(self, x: int = 0, y: int = 0, cost: float = 0.0):
        self.x = x
        self.y = y
        self.cost = cost

def add_coordinates(node1: Node, node2: Node):
    new_node = Node()
    new_node.x = node1.x + node2.x
    new_node.y = node1.y + node2.y
    new_node.cost = node1.cost + node2.cost
    return new_node

def compare_coordinates(node1: Node, node2: Node):
    return node1.x == node2.x and node1.y == node2.y

def distance(node1: Node, node2: Node):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

class DStarLite:

    motions = [
        Node(1, 0, 1),
        Node(0, 1, 1),
        Node(-1, 0, 1),
        Node(0, -1, 1),
        Node(1, 1, math.sqrt(2)),
        Node(1, -1, math.sqrt(2)),
        Node(-1, 1, math.sqrt(2)),
        Node(-1, -1, math.sqrt(2))
    ]

    def __init__(self, ox: list, oy: list):
        self.x_min_world = int(min(ox))
        self.y_min_world = int(min(oy))
        self.x_max = int(abs(max(ox) - self.x_min_world))
        self.y_max = int(abs(max(oy) - self.y_min_world))
        self.obstacles = [Node(x - self.x_min_world, y - self.y_min_world) for x, y in zip(ox, oy)]
        self.obstacles_xy = np.array(
            [[obstacle.x, obstacle.y] for obstacle in self.obstacles]
        )
        self.start = Node(0, 0)
        self.goal = Node(0, 0)
        self.U = []
        self.km = 0.0
        self.kold = 0.0
        self.rhs = self.create_grid(float("inf"))
        self.g = self.create_grid(float("inf"))
        self.detected_obstacles_xy = np.empty((0, 2))
        self.xy = np.empty((0, 2))
        self.initialized = False
        self.WP = []

    def create_grid(self, val: float):
        return np.full((self.x_max, self.y_max), val)
    
    def is_valid_position(self, node: Node, min_distance: float = 1.0) -> bool:
        """
        Check if the node position is valid by ensuring it's at least min_distance away from the nearest obstacle.

        :param node: The node to check.
        :param min_distance: The minimum required distance from the nearest obstacle.
        :return: True if the node is valid, False otherwise.
        """
        for obstacle in self.obstacles:
            if distance(node, obstacle) < min_distance:
                return False
        return True
    
    # def is_obstacle(self, node: Node):
    #     x = np.array([node.x])
    #     y = np.array([node.y])
    #     obstacle_x_equal = self.obstacles_xy[:, 0] == x
    #     obstacle_y_equal = self.obstacles_xy[:, 1] == y
    #     is_in_obstacle = (obstacle_x_equal & obstacle_y_equal).any()

    #     is_in_detected_obstacle = False
    #     if self.detected_obstacles_xy.shape[0] > 0:
    #         is_x_equal = self.detected_obstacles_xy[:, 0] == x
    #         is_y_equal = self.detected_obstacles_xy[:, 1] == y
    #         is_in_detected_obstacle = (is_x_equal & is_y_equal).any()

    #     return is_in_obstacle or is_in_detected_obstacle
    def is_obstacle(self, node: Node, min_distance: float = 2.5) -> bool:
        """
        Check if the node is considered an obstacle or is too close to an obstacle.

        :param node: The node to check.
        :param min_distance: The minimum distance a node must be from any obstacle to be considered valid.
        :return: True if the node is an obstacle or too close to one, False otherwise.
        """
        # Convert the node's coordinates to a numpy array for efficient distance computation
        node_xy = np.array([node.x, node.y])

        # Compute the distances from the node to all obstacles
        distances = np.sqrt(np.sum((self.obstacles_xy - node_xy) ** 2, axis=1))

        # Check if any distance is less than the minimum distance
        if np.any(distances < min_distance):
            return True  # The node is too close to an obstacle or is an obstacle

        return False  # The node is not an obstacle and respects the minimum distance requirement

    
    def c(self, node1: Node, node2: Node):
        if self.is_obstacle(node2):
            return math.inf
        new_node = Node(node1.x - node2.x, node1.y - node2.y)
        detected_motion = list(filter(lambda motion: compare_coordinates(motion, new_node), self.motions))
        return detected_motion[0].cost
    
    def h(self, s: Node):
        # Can be modified
        #return 1
        #return max(abs(self.start.x - s.x), abs(self.start.y - s.y)) # Manhattan distance
        # dx = abs(self.start.x - s.x)
        # dy = abs(self.start.y - s.y)
        # return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy) # Chebyshev distance
        return distance(s, self.goal) # Euclidean distance
    
    def calculate_key(self, s: Node):
        return (min(self.g[s.x][s.y], self.rhs[s.x][s.y]) + self.h(s) + self.km, min(self.g[s.x][s.y], self.rhs[s.x][s.y]))
    
    def is_valid(self, node: Node):
        if 0 <= node.x < self.x_max and 0 <= node.y < self.y_max:
            return True
        return False
    
    def get_neighbours(self, u: Node):
        return [add_coordinates(u, motion) for motion in self.motions if self.is_valid(add_coordinates(u, motion))]
    
    def pred(self, u: Node):
        # Grid, so each vertex is connected to the ones around it
        return self.get_neighbours(u)
    
    def succ(self, u: Node):
        return self.get_neighbours(u)
    
    def initialize(self, start: Node, goal: Node):
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
            self.detected_obstacles_xy = np.empty((0, 2))

    def update_vertex(self, u: Node):
        if not compare_coordinates(u, self.goal):
            self.rhs[u.x][u.y] = min([self.c(u, sprime) + self.g[sprime.x][sprime.y] for sprime in self.succ(u)])
        if any([compare_coordinates(u, node) for node, key in self.U]):
            self.U = [(node, key) for node, key in self.U if not compare_coordinates(node, u)]
            self.U.sort(key=lambda x: x[1])
        if self.g[u.x][u.y] != self.rhs[u.x][u.y]:
            self.U.append((u, self.calculate_key(u)))
            self.U.sort(key=lambda x: x[1])

    def get_direction(self, node1: Node, node2: Node):
        dx = node2.x - node1.x
        dx = dx/abs(dx) if dx != 0 else 0
        dy = node2.y - node1.y
        dy = dy/abs(dy) if dy != 0 else 0
        return dx, dy
    
    def detect_and_update_waypoints(self, current_point, next_point):
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

    def compare_keys(self, key_pair1: tuple[float, float], key_pair2: tuple[float, float]):
        return key_pair1[0] < key_pair2[0] or (key_pair1[0] == key_pair2[0] and key_pair1[1] < key_pair2[1])
    
    def compute_shortest_path(self):
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

    def detect_changes(self):
        changed_vertices = list()
        if len(self.spoofed_obstacles) > 0:
            for spoofed_obstacle in self.spoofed_obstacles[0]:
                if compare_coordinates(spoofed_obstacle, self.start) or \
                   compare_coordinates(spoofed_obstacle, self.goal):
                    continue
                changed_vertices.append(spoofed_obstacle)
                self.detected_obstacles_xy = np.concatenate(
                    (
                        self.detected_obstacles_xy,
                        [[spoofed_obstacle.x, spoofed_obstacle.y]]
                    )
                )
    
    def compute_current_path(self):
        path = list()
        current_point = Node(self.start.x, self.start.y)
        last_point = None

        while not compare_coordinates(current_point, self.goal):
            if last_point is not None:
                self.detect_and_update_waypoints(last_point, current_point)
            path.append(current_point)
            last_point = current_point
            current_point = min(self.succ(current_point), key = lambda sprime: self.c(current_point, sprime) + self.g[sprime.x][sprime.y])
        path.append(self.goal)
        self.WP.append(self.goal)
        return path
    
    def smooth_corner(self):
        pass
    
    def get_WP(self):
        WP_list = []
        for wp in self.WP:
            WP_list.append([wp.x + self.x_min_world, wp.y + self.y_min_world])
        return WP_list
    
    
    def main(self, start: Node, goal: Node): #, spoofed_ox: list, spoofed_oy: list):
        #self.spoofed_obstacles = [[Node(x - self.x_min_world, y - self.y_min_world) for x, y in zip(rowx, rowy)] for rowx, rowy in zip(spoofed_ox, spoofed_oy)]
        pathx = []
        pathy = []
        self.initialize(start, goal)
        last = self.start
        self.compute_shortest_path()
        pathx.append(self.start.x + self.x_min_world)
        pathy.append(self.start.y + self.y_min_world)
        print("Path found")
        return True, pathx, pathy

def main():
    # start and goal position
    sx = -5
    sy = 50
    gx = -5
    gy = 30

    # Set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(60)
    for i in range(20, 60):
        ox.append(60)
        oy.append(i)
    for i in range(-10, 60):
        ox.append(i)
        oy.append(60)
    for i in range(20, 61):
        ox.append(-10)
        oy.append(i)
    for i in range(-10, 60):
        ox.append(i)
        oy.append(55)
    for i in range(-10, 46):
        ox.append(i)
        oy.append(45)
    for i in range(35, 45):
        ox.append(45)
        oy.append(i)
    for i in range(-10, 46):
        ox.append(i)
        oy.append(35)
    for i in range(-10, 60):
        ox.append(i)
        oy.append(25)
    # for i in range(-10, 40):
    #     ox.append(20)
    #     oy.append(i)
    # for i in range(0, 40):
    #     ox.append(40)
    #     oy.append(60 - i)
    # for i in range(-10, 40):
    #     ox.append(15)
    #     oy.append(i)
    for i in range(-10, 60):
        ox.append(i)
        oy.append(20)
    
    dstarlite = DStarLite(ox, oy)
    dstarlite.main(Node(sx, sy), Node(gx, gy))
    path = dstarlite.compute_current_path()
    pathx = [node.x + dstarlite.x_min_world for node in path]
    pathy = [node.y + dstarlite.y_min_world for node in path]
    WP = np.array(dstarlite.get_WP())
    WP = np.array([[wp[1], wp[0]] for wp in WP])
    print("Waypoints: ", WP)
    new_WP = [WP[0]]
    for i in range(1, len(WP)):
        if abs(WP[i][0] - WP[i - 1][0]) < 2 and abs(WP[i][1] - WP[i - 1][1]) < 2:
            continue
        else:
            new_WP.append(WP[i])
    WP = np.array(new_WP)
    print("Waypoints: ", WP)
        
    hp = HybridPathGenerator(WP, 1, 0.3, 1)
    # Plotting
    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "og")
    plt.plot(gx, gy, "xb")
    plt.plot(pathx, pathy, "-r")
    for wp in WP:
        plt.plot(wp[1], wp[0], "or")
    plt.grid(True)
    plt.axis("equal")
    plt.show()

    hp.plot_path()


if __name__ == '__main__':
    main()
    
                            