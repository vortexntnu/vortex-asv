import math


class DSLNode:
    """Represents a DSLNode in the grid.

    Attributes:
        x (int): The x-coordinate of the DSLNode.
        y (int): The y-coordinate of the DSLNode.
        cost (float): The cost of moving to the DSLNode.

    """

    def __init__(self, x: int = 0, y: int = 0, cost: float = 0.0):
        """Initializes a new instance of the DSLNode class.

        Args:
            x (int): The x-coordinate of the DSLNode. Defaults to 0.
            y (int): The y-coordinate of the DSLNode. Defaults to 0.
            cost (float): The cost of moving to the DSLNode. Defaults to 0.0.
        """
        self.x = x
        self.y = y
        self.cost = cost

    def __add__(self, other: 'DSLNode') -> 'DSLNode':
        """Adds two DSLNode objects together.

        Args:
            other (DSLNode): The DSLNode to add to the current DSLNode.

        Returns:
            DSLNode: A new DSLNode object with the combined x and y coordinates and costs.
        """
        return DSLNode(self.x + other.x, self.y + other.y, self.cost + other.cost)

    def __eq__(self, other: 'DSLNode') -> bool:
        """Checks if two DSLNode objects have the same x and y coordinates.

        Args:
            other (DSLNode): The DSLNode to compare to the current DSLNode.

        Returns:
            bool: True if the nodes have the same x and y coordinates, False otherwise.
        """
        return self.x == other.x and self.y == other.y

    @staticmethod
    def distance_between_nodes(node1: 'DSLNode', node2: 'DSLNode') -> float:
        """Computes the Euclidean distance between two DSLNode objects.

        Args:
            node1 (DSLNode): The first DSLNode.
            node2 (DSLNode): The second DSLNode.

        Returns:
            float: The Euclidean distance between the two nodes.
        """
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    @staticmethod
    def get_direction(node1: 'DSLNode', node2: 'DSLNode') -> tuple:
        """Calculates the direction from node1 to node2.

        Args:
            node1 (DSLNode): The starting DSLNode.
            node2 (DSLNode): The ending DSLNode.

        Returns:
            tuple: A tuple of two integers representing the direction from node1 to node2.
        """
        dx = node2.x - node1.x
        dx = dx / abs(dx) if dx != 0 else 0
        dy = node2.y - node1.y
        dy = dy / abs(dy) if dy != 0 else 0
        return dx, dy
