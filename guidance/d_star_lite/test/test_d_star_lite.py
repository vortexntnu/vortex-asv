import unittest
from d_star_lite.d_star_lite import DStarLite
from d_star_lite.d_star_lite import combine_nodes, compare_coordinates, distance
from d_star_lite.d_star_lite import Node
import numpy as np

class TestDStarLite(unittest.TestCase):
    
    def setUp(self):
        # Create example nodes
        self.node1 = Node(0, 0, 2.0)
        self.node2 = Node(1, 1, 3.4)
        self.node5 = Node(-1, -1, -5.8)

        # Create example obstacle coordinates
        self.ox = [1, 2, 3, 4, 5]
        self.oy = [0, 0, 0, 0, 0]

        # Create example dsl object
        self.dsl = DStarLite(self.ox, self.oy)

    def tearDown(self):
        pass

    def test_combine_nodes(self):
        # Test the combine_nodes function
        result = combine_nodes(self.node1, self.node2)
        self.assertEqual(result.x, 1)
        self.assertEqual(result.y, 1)
        self.assertEqual(result.cost, 5.4)

        result = combine_nodes(self.node2, self.node5)
        self.assertEqual(result.x, 0)
        self.assertEqual(result.y, 0)
        self.assertEqual(result.cost, -2.4)

    def test_compare_coordinates(self):
        # Test the compare_coordinates function
        result = compare_coordinates(self.node1, self.node2)
        self.assertEqual(result, False)

        result = compare_coordinates(self.node1, self.node1)
        self.assertEqual(result, True)

    def test_distance(self):
        # Test the distance function
        result = distance(self.node1, self.node2)
        self.assertEqual(result, np.sqrt(2))

        result = distance(self.node2, self.node5)
        self.assertEqual(result, np.sqrt(8))

    def test_is_obstacle(self):
        # Test the is_obstacle function
        self.assertEqual(self.dsl.is_obstacle(Node(1, 0)), True)
        self.assertEqual(self.dsl.is_obstacle(Node(2, 0)), True)
        self.assertEqual(self.dsl.is_obstacle(Node(5, 0)), True)
        self.assertEqual(self.dsl.is_obstacle(Node(10, 0)), False)

    def test_movement_cost(self):
        # Test the movement_cost function
        self.assertEqual(self.dsl.movement_cost(Node(10, 0), Node(11, 0)), 1.0)
        self.assertEqual(self.dsl.movement_cost(Node(10, 0), Node(10, 1)), 1.0)
        self.assertEqual(self.dsl.movement_cost(Node(10, 10), Node(11, 11)), np.sqrt(2))
        self.assertEqual(self.dsl.movement_cost(Node(1, 0), Node(2, 0)), np.inf)

    def test_heuristic_distance(self):
        # Test the heuristic_distance function
        self.dsl.goal = Node(5, 5)
        self.assertEqual(self.dsl.heuristic_distance(Node(0, 0)), np.sqrt(50))
        self.assertEqual(self.dsl.heuristic_distance(Node(5, 5)), 0.0)
        self.assertEqual(self.dsl.heuristic_distance(Node(10, 10)), np.sqrt(50))

    def test_get_direction(self):
        # Test the get_direction function
        self.assertEqual(self.dsl.get_direction(Node(0, 0), Node(1, 0)), (1, 0))
        self.assertEqual(self.dsl.get_direction(Node(0, 0), Node(2, 2)), (1, 1))
        self.assertEqual(self.dsl.get_direction(Node(0, 0), Node(-1, 0)), (-1, 0))
        self.assertEqual(self.dsl.get_direction(Node(0, 0), Node(0, -1)), (0, -1))

if __name__ == '__main__':
    unittest.main()