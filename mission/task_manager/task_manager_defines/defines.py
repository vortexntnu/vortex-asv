#!/usr/bin/python3

from dataclasses import dataclass


@dataclass
class Task:
    id: int
    name: str


class Tasks:
    joystick = Task(id=0, name="joystick")
    collision_avoidance = Task(id=1, name="collision_avoidance")
