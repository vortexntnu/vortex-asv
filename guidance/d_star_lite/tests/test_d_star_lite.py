import unittest
import pytest
from d_star_lite.d_star_lite import DStarLite
from d_star_lite.d_star_lite_node import DSLNode
import numpy as np

class TestDStarLite(unittest.TestCase):
    
    def setUp(self):
        # Create example DSLNodes
        self.DSLNode1 = DSLNode(0, 0, 2.0)
        self.DSLNode2 = DSLNode(1, 1, 3.4)
        self.DSLNode5 = DSLNode(-1, -1, -5.8)

        # Create example obstacle coordinates
        self.ox = [1, 2, 3, 4, 5]
        self.oy = [0, 0, 0, 0, 0]
        
        # Create start and goal
        self.start = DSLNode(0, 0)
        self.goal = DSLNode(5, 5)

        # Create example dsl object
        self.dsl = DStarLite(self.ox, self.oy, self.start, self.goal)

    def tearDown(self):
        pass

    # Test node addition
    def test_combine_DSLNodes_with_positive_values(self):
        self.assertEqual(self.DSLNode1 + self.DSLNode2, DSLNode(1, 1, 5.4))

    def test_combine_DSLNodes_with_negative_values(self):
        self.assertEqual(self.DSLNode5 + self.DSLNode5, DSLNode(-2, -2, -11.6))

    def test_combine_DSLNodes_with_positive_and_negative_values(self):
        self.assertEqual(self.DSLNode1 + self.DSLNode5, DSLNode(-1, -1, -3.8))

    # Test node comparison
    def test_compare_coordinates_is_false_for_two_different_nodes(self):
        
        self.assertEqual(self.DSLNode1 == self.DSLNode2, False)
    
    def test_compare_coordinates_is_true_for_two_same_nodes(self):
        self.assertEqual(self.DSLNode1 == self.DSLNode1, True)

    # Test the distance function
    def test_distance_between_two_nodes_yields_correct_euclidean_distance(self):
        
        result = DSLNode.distance_between_nodes(self.DSLNode1, self.DSLNode2)
        self.assertEqual(result, np.sqrt(2))

        result = DSLNode.distance_between_nodes(self.DSLNode2, self.DSLNode5)
        self.assertEqual(result, np.sqrt(8))

    # Test the is_obstacle function
    def test_is_obstacle_is_true_when_node_is_obstacle(self):
        
        self.assertEqual(self.dsl.is_obstacle(DSLNode(1, 0)), True)

    def test_is_obstacle_is_true_when_node_is_in_safe_distance_from_obstacle(self):
        self.assertEqual(self.dsl.is_obstacle(DSLNode(2, 2)), True)

    def test_is_obstacle_is_false_when_node_is_not_obstacle(self):
        self.assertEqual(self.dsl.is_obstacle(DSLNode(10, 0)), False)

    # Test the movement_cost function
    def test_movement_cost_when_moving_straight(self):
        
        self.assertEqual(self.dsl.movement_cost(DSLNode(10, 0), DSLNode(11, 0)), 1.0)

    def test_movement_cost_when_moving_diagonally(self):
        self.assertEqual(self.dsl.movement_cost(DSLNode(10, 10), DSLNode(11, 11)), np.sqrt(2))

    def test_movement_cost_when_moving_to_obstacle(self):
        self.assertEqual(self.dsl.movement_cost(DSLNode(1, 0), DSLNode(2, 0)), np.inf)

    # Test the heuristic_distance function (distance to target node)
    def test_heuristic_distance(self):
        self.assertEqual(self.dsl.heuristic_distance(DSLNode(0, 0)), np.sqrt(50))
        self.assertEqual(self.dsl.heuristic_distance(DSLNode(5, 5)), 0.0)
        self.assertEqual(self.dsl.heuristic_distance(DSLNode(10, 10)), np.sqrt(50))
    # Test the get_direction function
    def test_get_direction_when_moving_in_positive_x_direction(self):
        self.assertEqual(DSLNode.get_direction(DSLNode(0, 0), DSLNode(1, 0)), (1, 0))

    def test_get_direction_when_moving_positive_diagonal(self):
        self.assertEqual(DSLNode.get_direction(DSLNode(0, 0), DSLNode(2, 2)), (1, 1))

    def test_get_direction_when_moving_in_negative_x_direction(self):
        self.assertEqual(DSLNode.get_direction(DSLNode(0, 0), DSLNode(-1, 0)), (-1, 0))

    def test_get_direction_when_moving_in_negative_y_direction(self):
        self.assertEqual(DSLNode.get_direction(DSLNode(0, 0), DSLNode(0, -1)), (0, -1))

    def test_get_direction_when_moving_in_negative_diagonal(self):
        self.assertEqual(DSLNode.get_direction(DSLNode(0, 0), DSLNode(-2, -2)), (-1, -1))

if __name__ == '__main__':
    unittest.main()