"""
Shared setup for the hardware check scripts: building a robot (real or fake),
logging every distance reading, and asking before anything moves.

Run the scripts from the repo root with the Pimoroni virtualenv's Python, for
example: ~/.virtualenvs/pimoroni/bin/python scripts/wall_test.py
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scout.fake_robot import FakeTrilobot  # noqa: E402
from scout.motion import Mover, Settings  # noqa: E402
from scout.robot_logic import is_valid_distance  # noqa: E402


class ReadingLog:
    """Wraps a robot and records every raw read_distance() result, so odd
    readings (such as a lost echo coming back as a huge distance) can be seen."""

    def __init__(self, robot, now):
        self._robot = robot
        self._now = now
        self.readings = []  # (time, cm)

    def read_distance(self, *args, **kwargs):
        reading = self._robot.read_distance(*args, **kwargs)
        self.readings.append((self._now(), reading))
        return reading

    def __getattr__(self, name):
        return getattr(self._robot, name)


def build(fake_obstacle_cm=None, settings=None):
    """A (robot, mover, log) triple. With fake_obstacle_cm set, the robot is a
    FakeTrilobot that far from a wall, in simulated time; otherwise it is the
    real Trilobot."""
    settings = settings or Settings()
    if fake_obstacle_cm is not None:
        robot = FakeTrilobot(obstacle_cm=fake_obstacle_cm,
                             max_speed_cm_s=settings.cm_per_s / settings.speed,
                             read_time_s=settings.sensor_read_s)
        sleep, now = robot.sleep, robot.now
    else:
        from trilobot import Trilobot  # only here: constructing it touches the hardware
        robot = Trilobot()
        sleep, now = time.sleep, time.monotonic
    log = ReadingLog(robot, now)
    return robot, Mover(log, settings, sleep=sleep, now=now), log


def confirm(message):
    print(message)
    answer = input("Type 'yes' to start, anything else to cancel: ")
    return answer.strip().lower() == 'yes'


def describe(reading):
    if reading is None:
        return "none"
    if not is_valid_distance(reading):
        return "no echo"
    return f"{reading:.1f} cm"


def positive(kind):
    """An argparse type that accepts only values above zero, converted with kind."""
    def parse(text):
        value = kind(text)
        if value <= 0:
            raise argparse.ArgumentTypeError(f"must be more than 0, not {text}")
        return value
    return parse
