#!/usr/bin/python3

import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from pid_controller import ssa
from std_msgs.msg import Float64MultiArray

import vessel

import rospy
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3, Wrench
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

DT = 0.1


class VesselVisualizer:

    def __init__(self, vessel):

        rospy.init_node('vessel_simulator')

        rospy.Subscriber("/thrust/force_input", Wrench, self.wrench_callback)
        rospy.Subscriber("/controller/lqr/setpoints", Float64MultiArray,
                    self.setpoint_callback)
        self.vessel = vessel

        self.fig, self.axes = plt.subplots(nrows=3, ncols=2, figsize=(10, 10))
        self.ax_vessel = self.axes[0, 0]
        self.ax_x = self.axes[0, 1]
        self.ax_y = self.axes[1, 0]
        self.ax_psi = self.axes[1, 1]
        self.ax_vx = self.axes[2, 0]
        self.ax_vy = self.axes[2, 1]

        x, y, psi, vx, vy, vpsi = self.vessel.state
        self.external_control_signal = np.zeros(3)
        self.u = np.zeros(3)
        self.brownian_motion_state = np.zeros(2)

        psi_north = psi + np.pi/2

        self.arrow = plt.Arrow(x, y, np.cos(psi_north), np.sin(psi_north), width=0.1)
        self.ax_vessel.add_patch(self.arrow)

        # Initialize the path line
        self.path, = self.ax_vessel.plot(x, y, color="red")


        self.setpoints = np.zeros(6)
        self.sp_x_line, = self.ax_x.plot(0, self.setpoints[0], color="r")
        self.sp_y_line, = self.ax_y.plot(0, self.setpoints[1], color="r")
        self.sp_psi_line, = self.ax_psi.plot(0, self.setpoints[2], color="r")
        self.sp_vx_line, = self.ax_vx.plot(0, self.setpoints[3], color="r")
        self.sp_vy_line, = self.ax_vy.plot(0, self.setpoints[4], color="r")
        
        self.window_size = 50

        self.ax_vessel.set_xlim(-5, 5)
        self.ax_vessel.set_ylim(-5, 5)
        self.ax_vessel.grid("on")

        self.x_line, = self.ax_x.plot(0, x)
        self.y_line, = self.ax_y.plot(0, y)
        self.psi_line, = self.ax_psi.plot(0, ssa(psi))
        self.vx_line, = self.ax_vx.plot(0, vx)
        self.vy_line, = self.ax_vy.plot(0, vy)

        self.current_time = 0.0

        self.odom_pub = rospy.Publisher("/odometry/filtered",
                                        Odometry,
                                        queue_size=10)
        
    def setpoint_callback(self, msg):
        #rospy.loginfo(f"Simulator received setpoints: {msg.data}")
        number_of_setpoints = len(msg.data)
        if number_of_setpoints != 6:
            return

        self.setpoints = np.array(msg.data)

    def wrench_callback(self, data):
        x_scale = 1.0
        y_scale = 1.0
        yaw_scale = 1.0
        self.u = np.array((x_scale * data.force.x, y_scale * data.force.y,
                           yaw_scale * data.torque.z))

    def update(self, frame):
        self.update_time_and_motion_state()
        #random_external_noise = np.append(self.brownian_motion_state, 0.0)
        self.vessel.step(DT, self.u)
        self.update_and_draw_arrow()
        self.update_path_line()
        self.update_and_plot_signals(frame)
        self.update_legends_and_axes(frame)

        odom = Odometry()

        # Assuming self.vessel.state is in the order: [x, y, yaw, vx, vy, vyaw]
        odom.pose.pose.position = Point(self.vessel.state[0],
                                        self.vessel.state[1], 0)
        quaternion = quaternion_from_euler(0, 0, self.vessel.state[2])
        odom.pose.pose.orientation = Quaternion(*quaternion)
        odom.twist.twist = Twist(
            Vector3(self.vessel.state[3], self.vessel.state[4], 0),
            Vector3(0, 0, self.vessel.state[5]))

        # Publish the message
        self.odom_pub.publish(odom)

    def update_time_and_motion_state(self):
        self.current_time += DT
        self.brownian_motion_state += np.sqrt(DT) * np.random.normal(scale=1.0,
                                                                     size=2)

    def update_and_draw_arrow(self):
        self.arrow.remove()
        arrow_length = 1.0
        arrow_width = 1.0
        x, y, psi, vx, vy, vpsi = self.vessel.state
        psi = ssa(psi)


        psi_north = ssa(-psi + np.pi/2)
        self.arrow = plt.Arrow(y,
                               x,
                               arrow_length * np.cos(psi_north),
                               arrow_length * np.sin(psi_north),
                               width=arrow_width)
        self.ax_vessel.add_patch(self.arrow)

    def update_path_line(self):
        window_size = 1000
        old_path = self.path.get_data()
        new_path = (np.append(old_path[0][-window_size:], self.vessel.state[1]),
                    np.append(old_path[1][-window_size:], self.vessel.state[0]))
        self.path.set_data(new_path)

    def update_and_plot_signals(self, frame):
        signals = ['x', 'y', 'psi', 'vx', 'vy']
        for i, signal in enumerate(signals):
            data = self.vessel.state[i]
            if signal == "psi":
                data = ssa(data)
            line = getattr(self, f"{signal}_line")

            # append the new data point and only keep the last 'self.window_size' points
            x_data = np.append(line.get_xdata()[-self.window_size:], frame)
            y_data = np.append(line.get_ydata()[-self.window_size:], data)

            line.set_data(x_data, y_data)

            setpoint = self.setpoints[i]  # Fetch the corresponding setpoint

            # Update the setpoint line with new data point
            sp_line = getattr(self, f"sp_{signal}_line")
            sp_x_data = np.append(sp_line.get_xdata()[-self.window_size:], frame)
            sp_y_data = np.append(sp_line.get_ydata()[-self.window_size:], setpoint)
            sp_line.set_data(sp_x_data, sp_y_data)

    def update_legends_and_axes(self, frame):
        signals = ['x', 'y', 'psi', 'vx', 'vy']
        labels = ['North', 'East', 'Heading', 'Surge Velocity', 'Sway Velocity']
        y_padding = 0.25

        self.ax_vessel.set_ylabel("North [m]")
        self.ax_vessel.set_xlabel("East [m]")

        for i, (signal, label) in enumerate(zip(signals, labels)):
            line = getattr(self, f"{signal}_line")
            axis = getattr(self, f"ax_{signal}")

            line.set_label(label)

            axis.set_xlim(min(line.get_xdata()), max(line.get_xdata()))
            
            axis.set_ylim(
                min(min(line.get_ydata()), self.setpoints[i]) - y_padding,
                max(max(line.get_ydata()), self.setpoints[i]) + y_padding)
            
            axis.legend()


    def animate(self):
        NUMBER_OF_FRAMES = 2000
        TIME_BETWEEN_FRAMES_MS = 50
        anim = FuncAnimation(self.fig,
                             self.update,
                             frames=np.arange(0, NUMBER_OF_FRAMES, DT),
                             interval=TIME_BETWEEN_FRAMES_MS,
                             repeat=False)
        plt.show()


if __name__ == '__main__':

    rospy.init_node("vessel_simulator")
    # # Initiate the vessel
    mass = 50.0
    inertia = 5.0

    damping_x = 5.0
    damping_y = 20.0
    damping_psi = 15.0

    # M = mass, mass, inertia
    # D = damping x, y, yaw
    M = np.diag([mass, mass, inertia])
    D = np.diag([damping_x, damping_y, damping_psi])
    simulated_vessel = vessel.Vessel3DOF(mass, damping_x, damping_y,
                                         damping_psi, inertia)

    # Create a visualizer and animate
    visualizer = VesselVisualizer(simulated_vessel)
    visualizer.animate()
