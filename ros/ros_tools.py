#!/usr/bin/env python3
import os
import sys
import subprocess
from pathlib import Path


def restart_with_ros_env():
    """
    Restarts the current Python process with ROS environment variables.
    Returns False if already running in ROS environment, True if restart initiated.
    """
    if 'ROS_RESTARTED' in os.environ:
        return False

    # Get ROS environment
    ros_setup = '/opt/ros/foxy/setup.bash'
    command = f"bash -c 'source {ros_setup} && env'"
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
    output = proc.communicate()[0].decode()

    # Parse environment variables
    env = {}
    for line in output.split('\n'):
        if line and '=' in line:
            key, value = line.split('=', 1)
            env[key] = value

    # Mark that we've restarted to prevent infinite loop
    env['ROS_RESTARTED'] = '1'

    # Replace current process with new one with ROS environment
    python = sys.executable
    os.execve(python, [python] + sys.argv, env)


def ensure_ros_environment():
    """
    Ensures ROS environment is properly set up.
    Call this at the start of your ROS Python scripts.
    """
    if restart_with_ros_env():
        # If restart_with_ros_env returns True, it means we're restarting
        # and this line won't actually be reached
        pass
    # If we get here, we're either in the restarted process
    # or already had ROS environment set up