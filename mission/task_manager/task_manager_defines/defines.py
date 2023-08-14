#!/usr/bin/python3

from dataclasses import dataclass


# Defines a type of object, Task, that has the attributes id and name
@dataclass
class Task:
    id: int
    name: str


# Class containing all tasks that can be chosen in gui set up using dynamic reconfigure
class Tasks:
    joystick = Task(id=0, name="joystick")
    collision_avoidance = Task(id=1, name="collision_avoidance")
    sea_marker_task1 = Task(id=2, name="sea_marker_task1")
    sea_marker_task2 = Task(id=3, name="sea_marker_task2")
    sea_marker_task3 = Task(id=4, name="sea_marker_task3")
