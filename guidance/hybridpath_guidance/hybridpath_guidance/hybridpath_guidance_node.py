#!/usr/bin/env python3

import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose2D
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from vortex_msgs.action import HybridpathGuidance
from vortex_msgs.msg import HybridpathReference
from vortex_utils.python_utils import quat_to_euler

from hybridpath_guidance.hybridpath import HybridPathGenerator, HybridPathSignals

qos_profile = QoSProfile(
    depth=1,
    history=qos_profile_sensor_data.history,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)


class HybridpathGuidanceNode(Node):
    def __init__(self):
        super().__init__("hp_guidance")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.action_server_ = ActionServer(
            self,
            HybridpathGuidance,
            'hybridpath_guidance',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.init_parameters()
        self.init_subscriber()
        self.init_publisher()
        self.init_hybridpath()

        self.get_logger().info("Hybridpath guidance node initialized")

    def init_parameters(self):
        hp_params = ['lambda_val', 'path_generator_order', 'dt', 'mu']

        for param in hp_params:
            self.declare_parameter(f'hybridpath_guidance.{param}', 0.0)
            setattr(
                self,
                param + '_',
                self.get_parameter(f'hybridpath_guidance.{param}').value,
            )

        topics = ['wrench_input', 'hp_guidance', 'odom']

        for topic in topics:
            self.declare_parameter(f'topics.{topic}', '_')

    def init_subscriber(self):
        self.odom_sub_ = self.create_subscription(
            Odometry,
            self.get_parameter('topics.odom').value,
            self.odom_callback,
            qos_profile=qos_profile,
        )

    def init_publisher(self):
        self.guidance_publisher_ = self.create_publisher(
            HybridpathReference, self.get_parameter('topics.hp_guidance').value, 1
        )

    def init_hybridpath(self):
        self.current_state_ = np.zeros(3)
        self.path_generator_ = HybridPathGenerator(
            self.path_generator_order_, self.lambda_val_
        )
        self.signals_ = HybridPathSignals()
        self.u_desired_ = 0.5
        self.path_ = None
        self.s_ = 0.0
        self.w_ = 0.0
        self.v_s_ = 0.0
        self.v_ss_ = 0.0

    def odom_callback(self, msg: Odometry):
        self.current_state_ = self.odom_to_eta(msg)

    def goal_callback(self, goal_request: HybridpathGuidance.Goal):
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and accept new goal")
                self.goal_handle_.abort()

        if len(goal_request.waypoints.waypoints) < 1:
            self.get_logger().info("Reject goal, no waypoints")
            return GoalResponse.REJECT

        self.get_logger().info("Accept new goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_ == goal_handle:
                self.get_logger().info("Cancel goal")
                return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        waypoint_list: list[Point] = goal_handle.request.waypoints.waypoints
        current_point = Point(x=self.current_state_[0], y=self.current_state_[1])
        waypoint_list.insert(0, current_point)

        self.reset_path(waypoint_list)

        result = HybridpathGuidance.Result()
        feedback = HybridpathGuidance.Feedback()

        self.get_logger().info("Executing goal")
        while rclpy.ok():
            if not goal_handle.is_active:
                result.success = False
                return result

            if goal_handle.is_cancel_requested:
                result.success = False
                goal_handle.canceled()
                return result

            if self.path_ is not None and self.s_ >= self.path_.NumSubpaths:
                hp_msg = self.create_hybridpath_message(final_point=True)
                self.guidance_publisher_.publish(hp_msg)
                goal_handle.succeed()
                result.success = True
                self.get_logger().info("Goal succeeded")
                return result

            self.update_path()

            hp_msg = self.create_hybridpath_message()

            feedback.feedback = hp_msg
            goal_handle.publish_feedback(feedback)

            self.guidance_publisher_.publish(hp_msg)

            time.sleep(self.dt_)

    def create_hybridpath_message(self, final_point=False) -> HybridpathReference:
        if final_point:
            desired_position = self.path_generator_.waypoints[-1]
            desired_position_derivative = [0.0, 0.0]
            desired_position_double_derivative = [0.0, 0.0]

            desired_heading = self.current_state_[2]
            desired_heading_derivative = 0.0
            desired_heading_double_derivative = 0.0
        else:
            desired_position = self.signals_.get_position()
            desired_position_derivative = self.signals_.get_derivatives()[0]
            desired_position_double_derivative = self.signals_.get_derivatives()[1]

            desired_heading = self.signals_.get_heading()
            desired_heading_derivative = 0.0
            desired_heading_double_derivative = 0.0

        hp_msg = HybridpathReference()
        hp_msg.eta_d = Pose2D(
            x=desired_position[0], y=desired_position[1], theta=desired_heading
        )
        hp_msg.eta_d_s = Pose2D(
            x=desired_position_derivative[0],
            y=desired_position_derivative[1],
            theta=desired_heading_derivative,
        )
        hp_msg.eta_d_ss = Pose2D(
            x=desired_position_double_derivative[0],
            y=desired_position_double_derivative[1],
            theta=desired_heading_double_derivative,
        )

        hp_msg.w = self.w_
        hp_msg.v_s = self.v_s_
        hp_msg.v_ss = self.v_ss_

        return hp_msg

    def reset_path(self, waypoint_list: list[Point]):
        self.path_generator_.create_path(waypoint_list)
        self.path_ = self.path_generator_.get_path()
        self.s_ = 0.0
        self.signals_.update_path(self.path_)
        self.w_ = self.signals_.get_w(self.mu_, self.current_state_)

    def update_path(self):
        self.s_ = self.path_generator_.update_s(
            self.path_, self.dt_, self.u_desired_, self.s_, self.w_
        )
        self.signals_.update_s(self.s_)
        self.w_ = self.signals_.get_w(self.mu_, self.current_state_)
        self.v_s_ = self.signals_.get_vs(self.u_desired_)
        self.v_ss_ = self.signals_.get_vs_derivative(self.u_desired_)

    @staticmethod
    def odom_to_eta(msg: Odometry) -> np.ndarray:
        """Converts an Odometry message to 3DOF eta vector.

        Args:
            msg (Odometry): The Odometry message to convert.

        Returns:
            np.ndarray: The eta vector.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        euler = quat_to_euler(x=q.x, y=q.y, z=q.z, w=q.w)

        heading = euler[2]

        state = np.array([x, y, heading])

        return state


def main(args=None):
    rclpy.init(args=args)
    node = HybridpathGuidanceNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
