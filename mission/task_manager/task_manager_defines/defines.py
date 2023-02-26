#!/usr/bin/python3

from dataclasses import dataclass


@dataclass
class Task:
    id: int
    name: str


class Tasks:
    joystick = Task(id=0, name="joystick")
    collision_avoidance = Task(id=1, name="collision_avoidance")
    ManeuveringNavigationTasks = Task(id=2, name="ManeuveringNavigationTasks")

    tasks = [joystick, collision_avoidance, ManeuveringNavigationTasks]
