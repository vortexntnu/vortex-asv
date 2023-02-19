#!/usr/bin/python3

import rospy
import dynamic_reconfigure.client


def callback(config):
    rospy.loginfo("Config set to {Njord_states}".format(**config))


if __name__ == "__main__":
    rospy.init_node("task_manager_client")
    rospy.loginfo("initializing client")

    task_manager_client = dynamic_reconfigure.client.Client(
        "/task_manager/server", timeout=3, config_callback=callback
    )
    rospy.loginfo("client initialized")

    rospy.spin()
