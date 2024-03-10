import pytest
from unittest.mock import Mock
from hybridpath_controller.hybridpath_controller_node import HybridPathControllerNode
from vortex_msgs.msg import HybridpathReference
from geometry_msgs.msg import Pose2D
import numpy as np
import rclpy

class TestHybridPathControllerNode:
    @pytest.fixture
    def node():
        rclpy.init()
        node = HybridPathControllerNode()
        node.wrench_publisher_ = Mock()
        node.AB_controller_ = Mock()
        yield node  # yield the node for testing
        rclpy.shutdown()  # make sure to shutdown rclpy after the test

    def test_control_law(self, node):
        # Arrange
            msg = HybridpathReference()
            msg.w_ref = 0.03819000726434428
            msg.v_ref = 0.1001407579811512
            msg.v_ref_t = 0.0
            msg.v_ref_s = 0.0108627157420586

            msg.eta = Pose2D(x=1.0, y=1.0, theta=0.0)
            msg.nu = Pose2D(x=0.0, y=0.0, theta=0.0)
            msg.eta_d = Pose2D(x=10.00035914, y=0.24996664, theta=1.56654648)
            msg.eta_d_s = Pose2D(x=0.04243858, y=9.98585381, theta=-0.33009297)
            msg.eta_d_ss = Pose2D(x=3.29165665, y=-1.09721888, theta=-11.90686869)

            expected_tau = np.array([41.0, 2.25865772, 3.32448561])
            node.AB_controller_.control_law.return_value = expected_tau

            # Act
            node.hpref_cb(msg)

            # Assert
            node.AB_controller_.control_law.assert_called_once()
            node.wrench_publisher_.publish.assert_called_once()
            published_msg = node.wrench_publisher_.publish.call_args[0][0]
            print(published_msg)
            assert published_msg.force.x == expected_tau[0]
            assert published_msg.force.y == expected_tau[1]
            assert published_msg.torque.z == expected_tau[2]