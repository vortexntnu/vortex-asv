import sys

sys.path.insert(0, "/home/hannahcl/Documents/vortex/monkey_tracking/data_generation")
from scenarios import BaseScenario
from utility import time_from_step

from track_manager import TRACK_STATUS

import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np


def plot_with_estimates(scenario, measurements, ground_truths, estimates):

    end_time = time_from_step(scenario.k, scenario.config.dt)
    min_alpha = 0.5

    # Ground truths
    for target in ground_truths:
        track = target.track
        x = track[:, 1]
        y = track[:, 2]
        # alpha = 1 - np.vectorize(max)(min_alpha, 1 - track[:, 5] / end_time)
        alpha = None
        plt.scatter(x, y, alpha=alpha)

    # Measurements
    for measurements_at_t in measurements:

        # opacity based on time
        for measurement in measurements_at_t:
            # alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
            color = "r" if measurement.is_clutter else "k"
            plt.scatter(
                measurement.pos[0],
                measurement.pos[1],
                marker="x",
                color=color,
                # alpha=alpha,
            )

    for estimates_at_t in estimates:
        # alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
        color = "b"
        plt.scatter(
            estimates_at_t[0],
            estimates_at_t[1],
            marker="+",
            color=color,
            # alpha=alpha,
        )

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    plt.gca().set_aspect("equal")
    plt.grid()
    plt.show()


def plot_tentative_confirm_del(
    scenario,
    measurements,
    ground_truths,
    tentative_estimates,
    conf_estimates,
    tentative_del_estimates,
):

    end_time = time_from_step(scenario.k, scenario.config.dt)
    min_alpha = 0.5

    # Ground truths
    for target in ground_truths:
        track = target.track
        x = track[:, 1]
        y = track[:, 2]
        # alpha = 1 - np.vectorize(max)(min_alpha, 1 - track[:, 5] / end_time)
        alpha = None
        plt.scatter(x, y, alpha=alpha)

    # Measurements
    for measurements_at_t in measurements:

        # opacity based on time
        for measurement in measurements_at_t:
            alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
            color = "b" if measurement.is_clutter else "k"
            plt.scatter(
                measurement.pos[0],
                measurement.pos[1],
                marker="x",
                color=color,
                alpha=alpha,
            )

    for tentative_estimates_at_t in tentative_estimates:
        for tentative_estimates in tentative_estimates_at_t:
            # alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
            color = "y"
            plt.scatter(
                tentative_estimates[0],
                tentative_estimates[1],
                marker="+",
                color=color,
                # alpha=alpha,
            )

    for estimates_at_t in conf_estimates:
        # alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
        color = "g"
        plt.scatter(
            estimates_at_t[0],
            estimates_at_t[1],
            marker="+",
            color=color,
            # alpha=alpha,
        )

    for del_estimates_at_t in tentative_del_estimates:
        # alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
        color = "r"
        plt.scatter(
            del_estimates_at_t[0],
            del_estimates_at_t[1],
            marker="+",
            color=color,
            # alpha=alpha,
        )

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    plt.gca().set_aspect("equal")
    plt.grid()
    plt.show()


def plot_interactive(
    scenario,
    measurements,
    ground_truths,
    tentative_estimates,
    conf_estimates,
    tentative_del_estimates,
    estimate_status,
    wait_for_btn_press,
):

    plt.ion()

    end_time = time_from_step(scenario.k, scenario.config.dt)
    min_alpha = 0.5

    # Ground truths
    for target in ground_truths:
        track = target.track
        x = track[:, 1]
        y = track[:, 2]
        # alpha = 1 - np.vectorize(max)(min_alpha, 1 - track[:, 5] / end_time)
        alpha = None
        plt.scatter(x, y, alpha=alpha)

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    plt.gca().set_aspect("equal")
    plt.grid()
    plt.show()

    # Measurements
    k_ten = 0
    k_conf = 0
    k_del = 0
    for k in range(len(measurements)):

        # opacity based on time
        for measurement in measurements[k]:
            alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
            color = "r" if measurement.is_clutter else "k"

            plt.scatter(
                measurement.pos[0],
                measurement.pos[1],
                marker="x",
                color=color,
                alpha=alpha,
            )

        if estimate_status[k] == TRACK_STATUS.tentative_confirm:
            for tentative_estimate in tentative_estimates[k_ten]:
                # alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
                color = "y"

                plt.scatter(
                    tentative_estimate[0],
                    tentative_estimate[1],
                    marker="+",
                    color=color,
                    # alpha=alpha,
                )
            k_ten += 1

        if estimate_status[k] == TRACK_STATUS.confirmed:
            estimates_at_t = conf_estimates[k_conf]
            color = "g"

            plt.scatter(
                estimates_at_t[0],
                estimates_at_t[1],
                marker="+",
                color=color,
                # alpha=alpha,
            )
            k_conf += 1

        if estimate_status[k] == TRACK_STATUS.tentative_delete:
            del_estimates_at_t = tentative_del_estimates[k_del]
            color = "r"
            plt.scatter(
                del_estimates_at_t[0],
                del_estimates_at_t[1],
                marker="+",
                color=color,
                # alpha=alpha,
            )
            k_del += 1

        plt.draw()
        if wait_for_btn_press:
            plt.waitforbuttonpress()
        else:
            plt.pause(0.1)

    plt.show()


# def plot_pos_and_vel(estimates):
#     x = []
#     y = []
#     u = []
#     v = []
#     for i in range(len(estimates)):
#         x.append(estimates[i][0])
#         y.append(estimates[i][1])
#         u.append(estimates[i][2])
#         v.append(estimates[i][3])

#     fig, ax = plt.subplots(figsize =(14, 8))
#     ax.quiver(x, y, u, v)

#     # ax.xaxis.set_ticks([])
#     # ax.yaxis.set_ticks([])
#     # ax.axis([-0.3, 2.3, -0.3, 2.3])
#     # ax.set_aspect('equal')

#     ax.set_xlim(-100, 100)
#     ax.set_ylim(-100, 100)

#     plt.show()
