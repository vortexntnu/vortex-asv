from hybridpath_controller.hybridpath_controller_node import HybridPathControllerNode

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions

import unittest
import numpy as np
import pytest
import rclpy

@pytest.mark.rostest
def generate_test_description():
    hybridpath_controller_node = launch_ros.actions.Node(
            package='hybridpath_controller',
            executable='hybridpath_controller_node',
            name='hybridpath_controller_node',
            output='screen'
        )
      
    return (
        launch.LaunchDescription([
            hybridpath_controller_node,

            launch_testing.actions.ReadyToTest()
        ]),
        {
            'controller': hybridpath_controller_node
        }
    )

class TestHybridPathControllerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.node = HybridPathControllerNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_control_law(self):
        """
        WARNING! Expects following values used in AB_controller_:
        I = np.eye(3)
        kappa = 0.5

        K_1 = np.diag([10, 10, 10])
        self.K_1_tilde = K_1 + kappa*I
        self.K_2 = np.diag([60, 60, 30])
        self.tau_max = np.array([41.0, 50.0, 55.0]) # Må tilpasses thrusterne

        m = 50
        self.M = np.diag([m, m, m])
        self.D = np.diag([10, 10, 5])
        """
        w_ref = 0.03819000726434428
        v_ref = 0.1001407579811512
        v_ref_t = 0.0
        v_ref_s = 0.0108627157420586

        eta = np.array([1.0, 1.0, 0.0])
        nu = np.array([0.0, 0.0, 0.0])
        eta_d = np.array([10.00035914, 0.24996664, 1.56654648])
        eta_d_s = np.array([0.04243858, 9.98585381, -0.33009297])
        eta_d_ss = np.array([3.29165665, -1.09721888, -11.90686869])

        expected_tau = np.array([41.0, 2.25865772, 3.32448561])

        tau = self.node.AB_controller_.control_law(eta, nu, w_ref, v_ref, v_ref_t, v_ref_s, eta_d, eta_d_s, eta_d_ss) 
        
        print(tau)
        print(expected_tau)
        assert np.isclose(tau[0], expected_tau[0], atol=1e-8)
        assert np.isclose(tau[1], expected_tau[1], atol=1e-8)
        assert np.isclose(tau[2], expected_tau[2], atol=1e-8)