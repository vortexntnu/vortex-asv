#!/usr/bin/python3

import rospy
import dynamic_reconfigure.client
from task_manager_defines import defines


class Template_task:

    """
    This task is a template for how one can set up a real task with the task manager
    This task is not defined in the class Tasks in defines.py, as this is just an example
    to use as template and not to actually run

    The client task_manager_client subscribes to changes made in the gui set up by the server
    """

    def __init__(self):
        rospy.init_node("template_task_node")

        self.isEnabled = False

        task_manager_server = rospy.get_param("/task_manager/task_manager_Server")

        # Subscribing to changes in task manager gui
        task_manager_client = dynamic_reconfigure.client.Client(
            task_manager_server, timeout=5, config_callback=self.callback
        )

    def callback(self, config):
        rospy.loginfo("""Client: task change request: {Njord_tasks}""".format(**config))
        activated_task_id = config["Njord_tasks"]

        # The template_task is not a real task defined in the Tasks class
        if defines.Tasks.template_task.id == activated_task_id:
            self.isEnabled = True
        else:
            self.isEnabled = False
        print(f"isEnabled: {self.isEnabled} ")

        return config


if __name__ == "__main__":
    colav = Template_task()

    rospy.spin()
