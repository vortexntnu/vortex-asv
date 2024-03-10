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

    
    def test_remove_invalid_waypoint_from_list(self):
        # Add a different waypoint to the list
        self.node.waypoint_list.append([3.0, 4.0])

        # Create a mock request for removing a waypoint that doesn't exist in the list
        class NonexistentWaypointRequest:
            waypoint = [1.0, 2.0]  
        nonexistent_waypoint_request = NonexistentWaypointRequest()

        # Calls the remove method
        nonexistent_waypoint_response = self.node.remove_waypoint_from_list(nonexistent_waypoint_request)

        # Assert that success = False
        self.assertFalse(nonexistent_waypoint_response.success)
    
    def test_remove_waypoint_from_empty_list(self):
        # Create a mock request for removing a waypoint from an empty list
        class EmptyListRequest:
            waypoint = [1.0, 2.0]
        empty_list_request = EmptyListRequest()

        # Call the remove method
        empty_list_response = self.node.remove_waypoint_from_list(empty_list_request)

        # Assert that success = False
        self.assertFalse(empty_list_response.success)


    def test_remove_valid_waypoint_from_list(self):
        # Adds a few waypoints first
        self.node.waypoint_list.append([1.0, 2.0])
        self.node.waypoint_list.append([2.0, 2.0])
        self.node.waypoint_list.append([3.0, 2.0])
        self.node.waypoint_list.append([4.0, 2.0])

        #Mock requests
        class MockRequest1:
            waypoint = [1.0, 2.0]
        class MockRequest2:
            waypoint = [2.0, 2.0]
        class MockRequest3:
            waypoint = [3.0, 2.0]
        class MockRequest4:
            waypoint = [4.0, 2.0]


        # Case 1: Attempt to remove the first waypoint [1.0, 2.0]
        request1 = MockRequest1()
        response1 = self.node.remove_waypoint_from_list(request1)
        self.assertTrue(response1.success)

        # Case 2: Attempt to remove the second waypoint [2.0, 2.0]
        request2 = MockRequest2()
        response2 = self.node.remove_waypoint_from_list(request2)
        self.assertTrue(response2.success)

        # Case 3: Attempt to remove the third waypoint [3.0, 2.0]
        request3 = MockRequest3()
        response3 = self.node.remove_waypoint_from_list(request3)
        self.assertTrue(response3.success)

        # Case 4: Attempt to remove the fourth waypoint [4.0, 2.0]
        request4 = MockRequest4()
        response4 = self.node.remove_waypoint_from_list(request4)
        self.assertTrue(response4.success)

if __name__ == '__main__':
    unittest.main()    
