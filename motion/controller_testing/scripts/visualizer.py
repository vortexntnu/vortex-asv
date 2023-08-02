#!/usr/bin/python3

import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from vessel import Vessel3DOF
from pid_controller import PIDController3DOF

DT = 1.0


class VesselVisualizer:

    def __init__(self, vessel):
        self.vessel = vessel
        # Create subplots for vessel visualization and setpoints vs actual positions and yaw
        self.fig, self.axes = plt.subplots(nrows=2, ncols=2, figsize=(10, 10))
        self.ax_vessel = self.axes[0, 0]
        self.ax_x = self.axes[0, 1]
        self.ax_y = self.axes[1, 0]
        self.ax_psi = self.axes[1, 1]

        # initialize the plot with initial vessel position and heading
        x, y, psi = self.vessel.state[3:]
        self.arrow = plt.Arrow(x, y, np.cos(psi), np.sin(psi), width=0.1)
        self.ax_vessel.add_patch(self.arrow)

        # Initialize the path line
        self.path, = self.ax_vessel.plot(x, y, color="red")

        # # Set the x and y axis limits
        self.ax_vessel.set_xlim(-5, 5)
        self.ax_vessel.set_ylim(-5, 5)

        # Initialize x, y, and psi plots
        self.x_line, = self.ax_x.plot(0, x)
        self.y_line, = self.ax_y.plot(0, y)
        self.psi_line, = self.ax_psi.plot(0, psi)

        self.current_time = 0.0

    def update(self, frame):
        self.current_time += DT

        # Clear the previous arrow
        self.arrow.remove()

        # Update vessel state
        u = pid_controller.control(self.vessel.state[3:],
                                   0.1)  # assuming dt=0.1
        print(
            f"Time: {self.current_time} \n Setpoint: {pid_controller.setpoint} \n State: {self.vessel.state[3:]} \n Control signal: {u} \n"
        )
        self.vessel.step(DT, u)

        # Draw the new arrow
        x, y, psi = self.vessel.state[3:]
        arrow_length = 1.0
        arrow_width = 1.0
        self.arrow = plt.Arrow(x,
                               y,
                               arrow_length * np.cos(psi),
                               arrow_length * np.sin(psi),
                               width=arrow_width)
        self.ax_vessel.add_patch(self.arrow)

        # Update the path line
        old_path = self.path.get_data()
        new_path = (np.append(old_path[0], x), np.append(old_path[1], y))
        self.path.set_data(new_path)

        # Update x, y, and psi plots
        x_y_data = np.append(self.x_line.get_ydata(), x)
        y_y_data = np.append(self.y_line.get_ydata(), y)
        psi_y_data = np.append(self.psi_line.get_ydata(), psi)

        self.x_line.set_data(np.append(self.x_line.get_xdata(), frame),
                             x_y_data)
        self.y_line.set_data(np.append(self.y_line.get_xdata(), frame),
                             y_y_data)
        self.psi_line.set_data(np.append(self.psi_line.get_xdata(), frame),
                               psi_y_data)

        # Plot setpoints on x, y, and psi plots
        self.ax_x.axhline(y=pid_controller.setpoint[0], color='r')
        self.ax_y.axhline(y=pid_controller.setpoint[1], color='r')
        self.ax_psi.axhline(y=pid_controller.setpoint[2], color='r')

        y_padding = 0.25
        window_size = 500

        self.ax_x.set_xlim(max(0, frame - window_size), frame + 1)
        self.ax_y.set_xlim(max(0, frame - window_size), frame + 1)
        self.ax_psi.set_xlim(max(0, frame - window_size), frame + 1)

        self.ax_x.set_ylim(
            min(x_y_data) - y_padding,
            max(max(x_y_data), pid_controller.setpoint[0]) + y_padding)
        self.ax_y.set_ylim(
            min(y_y_data) - y_padding,
            max(max(y_y_data), pid_controller.setpoint[1]) + y_padding)
        self.ax_psi.set_ylim(
            min(psi_y_data) - y_padding,
            max(max(psi_y_data), pid_controller.setpoint[2]) + y_padding)

    def animate(self):
        NUMBER_OF_FRAMES = 200
        TIME_BETWEEN_FRAMES_MS = 50
        anim = FuncAnimation(self.fig,
                             self.update,
                             frames=np.arange(0, NUMBER_OF_FRAMES, DT),
                             interval=TIME_BETWEEN_FRAMES_MS,
                             repeat=False)
        plt.show()


if __name__ == '__main__':
    # Initiate the vessel
    vessel = Vessel3DOF(mass=10, damping=0, inertia=1)

    # Create the PID controller
    Kp = [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.001]]
    Ki = [0.000, 0.000, 0.000]
    Kd = [0.1, 0.1, 0.0]
    setpoint = [1, 1, 0.5]  # Desired x, y and heading
    pid_controller = PIDController3DOF(Kp, Ki, Kd, setpoint)

    # Create a visualizer and animate
    visualizer = VesselVisualizer(vessel)
    visualizer.animate()
