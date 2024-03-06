import rclpy
from scripts.waypoint_manager import WaypointManager
from vortex_msgs.srv import Waypoint
import time

class TestWaypointManager:
    def send_request(self, x, y):
        self.request.waypoint.x = x
        self.request.waypoint.y = y
        future = self.client.call_async(self.request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Waypoint added successfully')
            else:
                self.get_logger().warn('Failed to add waypoint')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
    
    def test_add_waypoint_manager(self):
        rclpy.init()
        waypoint_manager = WaypointManager()
        self.client = self.create_client(Waypoint, 'add_waypoint_to_list')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = Waypoint.Request()

        self.client.send_request(x=10, y=5)

        # Add a delay of 1 second (adjust as needed)
        time.sleep(5)

        assert waypoint_manager.waypoint_list.__contains__(x=10, y=5)
        rclpy.shutdown()
