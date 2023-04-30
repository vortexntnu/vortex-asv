#!/usr/bin/python3
import rospy

from dynamic_reconfigure.server import Server
from task_manager.cfg import NjordTasksConfig


def callback(config, level):
    return config


if __name__ == "__main__":
    rospy.init_node("task_manager_server", anonymous=False)

    srv = Server(NjordTasksConfig, callback)
    rospy.spin()
