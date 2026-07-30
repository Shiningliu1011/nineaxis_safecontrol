#!/usr/bin/env python3
"""Compatibility entry point for the MoveIt 2 / ROS 2 transition pipeline.

Run from a built workspace with ``ros2 run robot_safecontrol_moveit
plan_transition``.  Keeping this file allows the historical ``python3
src/plan_transition.py`` command to delegate to the same implementation.
"""

from robot_safecontrol_moveit.plan_transition import main


if __name__ == "__main__":
    raise SystemExit(main())
