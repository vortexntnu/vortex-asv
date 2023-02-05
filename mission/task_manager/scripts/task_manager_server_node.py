#!/usr/bin/python3
import rospy

from dynamic_reconfigure.server import Server
from task_manager.cfg import ASV_FSMConfig

from task_manager_defines import defines

import collision_avoidance
#import ../mission/joystick_interface


def callback(config, level):
    rospy.loginfo("""State change request: {Njord_states}""".format(**config))

    if config["ASV_states"] == defines.Tasks.joystick.id:
        # Stop all other nodes from last state.
        rospy.set_param("/tasks/collision_avoidance", False)
        # Start the node here
        rospy.set_param("/tasks/joystick", True)

        param = rospy.get_param("/tasks/joystick")
        rospy.loginfo("joystick mode started, %s", param)

    if config["ASV_states"] == defines.Tasks.collision_avoidance.id:
        # Stop all other nodes from last state.
        rospy.set_param("/tasks/joystick", False)
        # Start the node here
        rospy.set_param("/tasks/collision_avoidance", True)

        param = rospy.get_param("/tasks/joystick")
        rospy.loginfo("joystick mode started, %s", param)

    return config


if __name__ == "__main__":
    rospy.init_node("task_manager_server", anonymous=False)

    srv = Server(ASV_FSMConfig, callback)
    rospy.spin()
