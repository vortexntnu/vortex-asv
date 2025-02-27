import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

end_point = (0.0, 0.0)
threshold = 0.25


class FreyaCheckGoal(Node):
    def __init__(self):
        super().__init__('freya_check_goal')
        self.subscription = self.create_subscription(
            Odometry, '/seapath/odom/ned', self.odometry_callback, 10
        )

        self.current_odom: Odometry = None
        self.received_odom: bool = False

    def odometry_callback(self, msg):
        self.current_odom = msg
        self.received_odom = True


def main(args=None):
    rclpy.init(args=args)

    node = FreyaCheckGoal()

    print(f"Waiting for the robot to reach the goal at {end_point}...")

    start_time = time.time()
    timeout = 20  # seconds

    while rclpy.ok() and time.time() - start_time < timeout:
        rclpy.spin_once(node)
        if node.received_odom:
            x = node.current_odom.pose.pose.position.x
            y = node.current_odom.pose.pose.position.y
            distance = math.sqrt((x - end_point[0]) ** 2 + (y - end_point[1]) ** 2)
            if distance < threshold:
                node.get_logger().info('Goal reached :)')
                rclpy.shutdown()
                exit(0)
        time.sleep(0.1)

    print(
        f"Timeout reached. The robot did not reach the goal at {end_point}. Current position: ({x}, {y})"
    )
    rclpy.shutdown()
    exit(1)


if __name__ == '__main__':
    main()
