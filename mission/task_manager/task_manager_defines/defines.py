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
