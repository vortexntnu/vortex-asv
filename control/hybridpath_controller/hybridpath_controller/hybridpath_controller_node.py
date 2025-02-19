#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from vortex_msgs.msg import HybridpathReference

from hybridpath_controller.adaptive_backstep import AdaptiveBackstep

qos_profile = QoSProfile(
    depth=1,
    history=qos_profile_sensor_data.history,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)


class HybridPathControllerNode(Node):
    def __init__(self):
        super().__init__("hybridpath_controller_node")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('hybridpath_controller.K1_diag', [10.0, 10.0, 10.0]),
                ('hybridpath_controller.K2_diag', [60.0, 60.0, 60.0]),
                ('physical.inertia_matrix', [50.0, 50.0, 50.0]),
                ('physical.damping_matrix_diag', [10.0, 10.0, 5.0]),
            ],
        )

        self.get_topics()

        self.parameters_updated = False

        self.killswitch_active = False
        self.operational_mode = 'autonomous mode'

        self.state_subscriber_ = self.state_subscriber_ = self.create_subscription(
            Odometry, self.odom_topic, self.state_callback, qos_profile=qos_profile
        )
        self.hpref_subscriber_ = self.create_subscription(
            HybridpathReference,
            self.hp_guidance_topic,
            self.reference_callback,
            1,
        )
        self.wrench_publisher_ = self.create_publisher(Wrench, self.wrench_input_topic, 1)
        self.operational_mode_subscriber = self.create_subscription(
            String, self.operation_mode_topic, self.operation_mode_callback, 10
        )
        self.killswitch_subscriber = self.create_subscription(
            Bool, self.killswitch_topic, self.killswitch_callback, 10
        )

        self.backstepping_controller_ = AdaptiveBackstep()

        self.update_controller_parameters()

        controller_period = 0.1
        self.controller_timer_ = self.create_timer(
            controller_period, self.controller_callback
        )
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("hybridpath_controller_node started")

    def operation_mode_callback(self, msg: String):
        self.operational_mode = msg.data

    def killswitch_callback(self, msg: Bool):
        self.killswitch_active = msg.data

    def update_controller_parameters(self):
        k1_diag = (
            self.get_parameter('hybridpath_controller.K1_diag')
            .get_parameter_value()
            .double_array_value
        )
        k2_diag = (
            self.get_parameter('hybridpath_controller.K2_diag')
            .get_parameter_value()
            .double_array_value
        )
        d_diag = (
            self.get_parameter('physical.damping_matrix_diag')
            .get_parameter_value()
            .double_array_value
        )
        m = (
            self.get_parameter('physical.inertia_matrix')
            .get_parameter_value()
            .double_array_value
        )

        k1 = np.diag(k1_diag)
        k2 = np.diag(k2_diag)
        d = np.diag(d_diag)
        m = np.reshape(m, (3, 3))

        self.backstepping_controller_.update_parameters(k1, k2, m, d)

        self.get_logger().info("Updated controller parameters")

    def parameter_callback(self, params):
        self.parameters_updated = True
        for param in params:
            if param.name == 'hybridpath_controller.K1_diag':
                self.get_logger().info(f"Updated K1_diag to {param.value}")
            elif param.name == 'hybridpath_controller.K2_diag':
                self.get_logger().info(f"Updated K2_diag to {param.value}")
            elif param.name == 'physical.damping_matrix_diag':
                self.get_logger().info(f"Updated damping_matrix_diag to {param.value}")
            elif param.name == 'physical.inertia_matrix':
                self.get_logger().info(f"Updated inertia_matrix to {param.value}")

        # self.update_controller_parameters()
        return SetParametersResult(successful=True)
    
    def get_topics(self):
        topics = [
            "odom",
            "hp_guidance",
            "wrench_input",
            "killswitch",
            "operation_mode",
        ]

        for topic in topics:
            self.declare_parameter(f"topics.{topic}", "_")
            setattr(self, f"{topic}_topic", self.get_parameter(f"topics.{topic}").value)

    def state_callback(self, msg: Odometry):
        """Callback function for the Odometry message. This function saves the state message."""
        self.state_odom = msg

    def reference_callback(self, msg: HybridpathReference):
        self.reference = msg

    def controller_callback(self):
        """Callback function for the controller timer. This function calculates the control input and publishes the control input."""
        if self.killswitch_active or self.operational_mode != 'autonomous mode':
            return

        if self.parameters_updated:
            self.update_controller_parameters()
            self.parameters_updated = False

        if hasattr(self, 'state_odom') and hasattr(self, 'reference'):
            control_input = self.backstepping_controller_.control_law(
                self.state_odom, self.reference
            )
            wrench_msg = Wrench()
            wrench_msg.force.x = control_input[0]
            wrench_msg.force.y = control_input[1]
            wrench_msg.torque.z = control_input[2]
            self.wrench_publisher_.publish(wrench_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HybridPathControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
