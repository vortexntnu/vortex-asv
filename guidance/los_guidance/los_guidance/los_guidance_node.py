import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler, euler2quat
from los_guidance.los_guidance import LOSGuidance

class LOSGuidanceNode(Node):
    def __init__(self):
        super().__init__("los_guidance_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('los_guidance.p0', [0.0, 0.0]),
                ('los_guidance.p1', [0.0, 0.0]),
                ('los_guidance.look_ahead_distance', 0.0)
            ])
        
        p0 = self.get_parameter('los_guidance.p0').get_parameter_value().double_array_value
        p1 = self.get_parameter('los_guidance.p1').get_parameter_value().double_array_value
        self.look_ahead = self.get_parameter('los_guidance.look_ahead_distance').get_parameter_value().double_value
        
        self.get_logger().info(f"p0: {p0}")
        self.get_logger().info(f"p1: {p1}")
        self.get_logger().info(f"look_ahead_distance: {self.look_ahead}")

        self.los_guidance = LOSGuidance(p0, p1)
        

        self.guidance_publisher_ = self.create_publisher(Odometry, "controller/lqr/reference", 1)
        self.state_subscriber_ = self.create_subscription(Odometry, "/sensor/seapath/odometry/ned", self.state_cb, 1)

        self.get_logger().info("los_guidance_node started")
    
    def state_cb(self, msg):
        # self.get_logger().info("state_callback")

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
        ]

        # Convert quaternion to Euler angles
        (roll, pitch, yaw) = quat2euler(orientation_list)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vyaw = msg.twist.twist.angular.z

        state = np.array([x, y, yaw, vx, vy, vyaw])

        x_ref = self.los_guidance.calculate_LOS_x_ref(state, self.look_ahead)

        orientation_list_ref = euler2quat(roll, pitch, x_ref[2])
        
        odometry_msg = Odometry()
        odometry_msg.pose.pose.position.x = x_ref[0]
        odometry_msg.pose.pose.position.y = x_ref[1]
        odometry_msg.pose.pose.position.z = 0.0
        odometry_msg.pose.pose.orientation.w = orientation_list_ref[0]
        odometry_msg.pose.pose.orientation.x = orientation_list_ref[1]
        odometry_msg.pose.pose.orientation.y = orientation_list_ref[2]
        odometry_msg.pose.pose.orientation.z = orientation_list_ref[3]
        
        self.guidance_publisher_.publish(odometry_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LOSGuidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
