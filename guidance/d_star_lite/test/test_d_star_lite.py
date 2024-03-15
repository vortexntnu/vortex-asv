import pytest
import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions

import unittest
from unittest.mock import ANY
import numpy as np
import rclpy

from d_star_lite import d_star_lite_node
from vortex_msgs.srv import Waypoint

@pytest.mark.rostest
def generate_test_description():
    return (launch.LaunchDescription([
        launch_ros.actions.Node(
            package='d_star_lite',
            executable='d_star_lite_node.py',
            name='d_star_lite_node',
            output='screen'
        ),
        launch_testing.actions.ReadyToTest()
    ]),
    {
        'd_star_lite_node': d_star_lite_node
    })

class TestDStarLiteNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = d_star_lite_node.DStarLiteNode()
    
    def tearDown(self):
        self.node.destroy_node()

    def test_dstarlitenode_creation(self):
        node = d_star_lite_node.DStarLiteNode()
        assert isinstance(node, d_star_lite_node.DStarLiteNode)

    # def test_send_waypoints_request_service_not_available(self, mocker):
    #     node = d_star_lite_node.DStarLiteNode()
    #     mocker.patch.object(node.wp_client, 'wait_for_service', return_value=False)
    #     assert node.send_waypoints_request() is None

    # def test_send_waypoints_request_correct_waypoints(self, mocker):
    #     node = d_star_lite_node.DStarLiteNode()
    #     node.wp_client.call_async.assert_called_once_with(ANY)
    #     expected_request = Waypoint.Request()
    #     expected_request.waypoints = [1.0, 2.0]
    #     node.wp_client.call_async.assert_called_once_with(expected_request)

