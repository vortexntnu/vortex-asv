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
        
        self.m_ip_list = ["10.24.166.212"]
        
        self.m_force_publisher = self.create_publisher(Float32MultiArray, '/thrust/thruster_forces', 5)
        self.m_timer = self.create_timer(1.0, self.timer_callback)
        
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
        self.get_logger().info('Attempting to shutdown thruster allocator')
        
        client = self.create_client(ChangeState, '/motion/thruster_allocator_node/change_state')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available')
            return
        
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def publish_zero_force(self):
        self.get_logger().info('Publishing zero force command')
        message = Float32MultiArray()
        message.data = [0.0] * 4
        self.m_force_publisher.publish(message)
        self.get_logger().info('Zero force command published')

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
