#!/usr/bin/python3

import rospy
import dynamic_reconfigure.client
from task_manager_defines import defines

class Colav:
    def __init__(self):
        rospy.init_node("collision_avoidance")

        self.isEnabled = False


    def callback(self, config):
        rospy.loginfo("""Client: task change request: {Njord_tasks}""".format(**config))
        activated_task_id = config["Njord_tasks"]

        if defines.Tasks.collision_avoidance.id == activated_task_id:
            self.isEnabled = True
        else:
            self.isEnabled = False
        print(f"isEnabled: {self.isEnabled} ")

        return config



if __name__ == "__main__":
    colav = Colav()
    task_manager_client = dynamic_reconfigure.client.Client(
    "task_manager/task_manager_server", timeout=5, config_callback=Colav.callback)
    rospy.spin()