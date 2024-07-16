#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import os
import sys

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        self.declare_parameter('ip_list', rclpy.Parameter.Type.STRING_ARRAY)
        self.m_ip_list = self.get_parameter('ip_list').get_parameter_value().string_array_value

        self.declare_parameter("ping_rate", rclpy.Parameter.Type.DOUBLE)
        ping_rate = self.get_parameter("ping_rate").get_parameter_value()

        self.m_force_publisher = self.create_publisher(Float32MultiArray, '/thrust/thruster_forces', 5)
        self.m_timer = self.create_timer(1.0, self.timer_callback)

        self.m_allocator_lifecycle_client = self.create_client(ChangeState, '/motion/thruster_allocator_node/change_state')

        self.get_logger().info('SystemMonitor initialized')

    def timer_callback(self):
        all_ips_responsive = all(self.ping_ip(ip) for ip in self.m_ip_list)
        
        if not all_ips_responsive:
            self.get_logger().warn('One or more IPs are unresponsive. Initiating shutdown sequence.')
            self.shutdown_thruster_allocator()
            self.publish_zero_force()
            sys.exit(0)

    def ping_ip(self, ip):
        response = os.system(f"ping -c 1 -W 1 {ip} > /dev/null 2>&1")
        if response != 0:
            self.get_logger().error(f'Failed to ping IP: {ip}')
            return False
        return True

    def shutdown_thruster_allocator(self):
        self.get_logger().warn("Shutting down thruster allocator"
         " -> Manual intervention will be necessary")
        
        if not self.m_allocator_lifecycle_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error('Service not available')
            return
        
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        
        future = self.m_allocator_lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

    def publish_zero_force(self):
        self.get_logger().info('Publishing zero-force on shutdown...')
        message = Float32MultiArray()
        message.data = [0.0] * 4
        self.m_force_publisher.publish(message)
        self.get_logger().warn('Done! Manual intervention is now required!')


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    node.get_logger().info('Starting SystemMonitor node')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('SystemMonitor node shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
