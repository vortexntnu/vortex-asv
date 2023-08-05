#!/usr/bin/python3

import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from scipy.integrate import odeint

#import freya
from pid_controller import PIDController3DOF

import vessel

DT = 0.25


def get_2d_state(state):
    x = state[0]
    y = state[1]
    psi = state[5]
    vx = state[6]
    vy = state[7]
    return x, y, psi, vx, vy


class VesselVisualizer:

    def __init__(self, vessel):
        self.vessel = vessel

        self.fig, self.axes = plt.subplots(nrows=3, ncols=2, figsize=(10, 10))
        self.ax_vessel = self.axes[0, 0]
        self.ax_x = self.axes[0, 1]
        self.ax_y = self.axes[1, 0]
        self.ax_psi = self.axes[1, 1]
        self.ax_vx = self.axes[2, 0]
        self.ax_vy = self.axes[2, 1]

        vx, vy, vpsi, x, y, psi = self.vessel.state
        
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
        self.vx_line, = self.ax_vx.plot(0, vx)
        self.vy_line, = self.ax_vy.plot(0, vy)

        self.current_time = 0.0

    def update(self, frame):
        self.current_time += DT

        # Clear the previous arrow
        self.arrow.remove()


        u = pid_controller.control(self.vessel.state[3:], 0.1)  # assuming dt=0.1
        #u = np.array((0.1, 0, 0))
        self.vessel.step(DT, u)

        # Draw the new arrow
        arrow_length = 1.0
        arrow_width = 1.0
        vx, vy, vpsi, x, y, psi = self.vessel.state

        print(f"Time: {self.current_time} \n Setpoint: {pid_controller.setpoint} \n State: {x}, {y}, {psi} \n Control signal: {u} \n")

        self.arrow = plt.Arrow(x, y, arrow_length*np.cos(psi), arrow_length*np.sin(psi), width=arrow_width)
        self.ax_vessel.add_patch(self.arrow)

        # Update the path line
        old_path = self.path.get_data()
        new_path = (np.append(old_path[0], x), np.append(old_path[1], y))
        self.path.set_data(new_path)

        # Update x, y, and psi plots
        x_y_data = np.append(self.x_line.get_ydata(), x)
        y_y_data = np.append(self.y_line.get_ydata(), y)
        psi_y_data = np.append(self.psi_line.get_ydata(), psi)
        vx_y_data = np.append(self.vx_line.get_ydata(), vx)
        vy_y_data = np.append(self.vy_line.get_ydata(), vy)    

        self.x_line.set_data(np.append(self.x_line.get_xdata(), frame), x_y_data)
        self.y_line.set_data(np.append(self.y_line.get_xdata(), frame), y_y_data)
        self.psi_line.set_data(np.append(self.psi_line.get_xdata(), frame), psi_y_data)
        self.vx_line.set_data(np.append(self.vx_line.get_xdata(), frame), vx_y_data)
        self.vy_line.set_data(np.append(self.vy_line.get_xdata(), frame), vy_y_data)


        self.x_line.set_label('North')
        self.y_line.set_label('East')
        self.psi_line.set_label('Heading')
        self.vx_line.set_label('Surge Velocity')
        self.vy_line.set_label('Sway Velocity')

        # Plot setpoints on x, y, and psi plots
        self.ax_x.axhline(y=pid_controller.setpoint[0], color='r')
        self.ax_y.axhline(y=pid_controller.setpoint[1], color='r')
        self.ax_psi.axhline(y=pid_controller.setpoint[2], color='r')

        y_padding = 0.25
        window_size = 500

        self.ax_x.set_xlim(max(0, frame-window_size), frame+1)
        self.ax_y.set_xlim(max(0, frame-window_size), frame+1)
        self.ax_psi.set_xlim(max(0, frame-window_size), frame+1)
        self.ax_vx.set_xlim(max(0, frame-window_size), frame+1)
        self.ax_vy.set_xlim(max(0, frame-window_size), frame+1)

        self.ax_x.set_ylim(min(x_y_data) - y_padding, max(max(x_y_data), pid_controller.setpoint[0]) + y_padding)
        self.ax_y.set_ylim(min(y_y_data) - y_padding, max(max(y_y_data), pid_controller.setpoint[1]) + y_padding)
        self.ax_psi.set_ylim(min(psi_y_data) - y_padding, max(max(psi_y_data), pid_controller.setpoint[2]) + y_padding)
        # self.ax_vx.set_ylim(min(vx_y_data) - y_padding, max(max(vx_y_data), pid_controller.setpoint[3]) + y_padding)
        # self.ax_vy.set_ylim(min(vy_y_data) - y_padding, max(max(vy_y_data), pid_controller.setpoint[4]) + y_padding)

        self.ax_x.legend()
        self.ax_y.legend()
        self.ax_psi.legend()
        self.ax_vx.legend()
        self.ax_vy.legend()

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
        NUMBER_OF_FRAMES = 2000
        TIME_BETWEEN_FRAMES_MS = 50
        anim = FuncAnimation(self.fig,
                             self.update,
                             frames=np.arange(0, NUMBER_OF_FRAMES, DT),
                             interval=TIME_BETWEEN_FRAMES_MS,
                             repeat=False)
        plt.show()


if __name__ == '__main__':
    # # Initiate the vessel
    mass = 20.0
    damping = 2.0
    inertia = 1.0
    simulated_vessel = vessel.Vessel3DOF(mass, damping, inertia)

    N = 20  # prediction horizon

    # cost matrices
    Q = np.diag([1, 1, 1, 1, 1, 1])  # state error cost
    R = np.diag([0.1, 0.1, 0.1])  # control effort cost


    # Create the PID controller
    Kp = [[0.1, 0, 0], [0, 1.0, 0], [0, 0, 0.25]]
    Ki = [0.001, 0.001, 0.000]
    Kd = [0.0, 0.0, 0.0]
    setpoint = [1, 0, np.pi/4]  # Desired x, y and heading
    pid_controller = PIDController3DOF(Kp, Ki, Kd, setpoint)

    # Create a visualizer and animate
    visualizer = VesselVisualizer(simulated_vessel)
    visualizer.animate()
