import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from vortex_msgs.action import HybridpathGuidance
from vortex_msgs.msg import Waypoints


class HybridpathGuidanceClient(Node):
    def __init__(self):
        super().__init__('hybridpath_guidance_client')
        # Create the action client
        self._action_client = ActionClient(
            self, HybridpathGuidance, '/freya/hybridpath_guidance'
        )
        self.send_goal()

    def send_goal(self):
        goal_msg = HybridpathGuidance.Goal()

        points = [
            Point(x=10.0, y=0.0),
            Point(x=10.0, y=10.0),
            Point(x=0.0, y=10.0),
            Point(x=0.0, y=0.0),
        ]
        goal_msg.waypoints = Waypoints()
        goal_msg.waypoints.waypoints = points

        self._action_client.wait_for_server()
        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.feedback
        # self.get_logger().info(f'Received feedback: x={feedback.x}, y={feedback.y}')

    def get_result_callback(self, future):
        result = future.result().result.success
        self.get_logger().info(f'Goal result: {result}')
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    client = HybridpathGuidanceClient()

    rclpy.spin(client)


if __name__ == '__main__':
    main()
