# Task manager package
=======================

- The task manager package is a package containing functionality for setting up a gui using dynamic reconfigure.

- The gui lets the user choose a task from a drop down menu, and is defined by the task_manager.cfg config file

- The server defined in task_manager_server_node sets up the gui according to the config file, and listens for changes made in the gui by the user.

- A template node for how one can set up a task as a client, so that it can be used together with the task manager, is defined in the folder test_nodes. This node is not runnable, as it tries to access a task that is not defined, and is therefore only meant to be used as a guide.

- For simplicity, when it comes to defining a task, a python package is made. The package contains a python dataclass object called Task, which defines a task by it's name and id (the id is only necessary for the gui setup). All tasks are then grouped together witin a class Tasks. This way, the name (and id) of a task only has to be defined one place, meaning that when the task is added in the config file and the task node is integrated with the task manager, one can simply access the task name through the python package in both cases.