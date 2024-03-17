import rclpy
import unittest
from scripts.waypoint_manager import WaypointManager
from vortex_msgs.srv import Waypoint

class TestWaypointManager(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = WaypointManager()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_add_waypoint_to_list(self):
        # Mocking a request object
        class MockRequest1:
            waypoint = [1.0, 2.0]
        class MockRequest2:
            waypoint = [2.0, 3.1]

        request1 = MockRequest1()
        request2 = MockRequest2()

        # Call the method
        response1 = self.node.add_waypoint_to_list(request1)

        # Check if the waypoint is added
        self.assertTrue(response1.success)
        self.assertIn([1.0, 2.0], self.node.waypoint_list)

        # Check that a waypoint that has not been added is not in the list
        self.assertNotIn([2.0, 3.1], self.node.waypoint_list)

        # Adds and checks if the waypoint is added
        response2 = self.node.add_waypoint_to_list(request2)
        self.assertTrue(response2.success)
        self.assertIn([2.0, 3.1], self.node.waypoint_list)


if __name__ == '__main__':
    unittest.main()    
