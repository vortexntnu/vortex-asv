import rclpy
from std_msgs.msg import Float64MultiArray, Wrench
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler, euler2quat
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are
# Assuming your_module.py is in the same package as lqr_controller
from lqr_controller.lqr_controller import LQRController
from lqr_controller.asv_model import ASV


def ssa(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class VesselVisualizer:
    # ... (unchanged)

    def update(self, frame):
        # ... (unchanged)

        # Assuming self.vessel.state is in the order: [x, y, yaw, vx, vy, vyaw]
        odom = Odometry()
        odom.pose.pose.position = Point(self.vessel.state[0], self.vessel.state[1], 0)
        quaternion = quaternion_from_euler(0, 0, self.vessel.state[2])
        odom.pose.pose.orientation = Quaternion(*quaternion)
        odom.twist.twist = Twist(Vector3(self.vessel.state[3], self.vessel.state[4], 0),
                                Vector3(0, 0, self.vessel.state[5]))
        self.odom_pub.publish(odom)

    # ... (unchanged)

if __name__ == '__main__':
    rclpy.init()

    # Initiate the vessel
    mass = 50.0
    damping_x = 5.0
    damping_y = 20.0
    damping_psi = 15.0
    inertia = 5.0

    M = np.diag([mass, mass, inertia])
    D = np.diag([damping_x, damping_y, damping_psi])
    lqr_controller = LQRController(mass, [damping_x, damping_y, damping_psi], [10.0, 10.0, 5.0, 0.1, 1.0, 5.0], [1.0, 1.0, 1.0])
    simulated_vessel = ASV(M, D)

    visualizer = VesselVisualizer(simulated_vessel)
    visualizer.animate()

    rclpy.spin()
    rclpy.shutdown()
