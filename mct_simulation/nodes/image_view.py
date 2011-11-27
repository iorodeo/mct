#!/usr/bin/python
from __future__ import division
import roslib
roslib.load_manifest('mct_simulation')
import rospy
import subprocess
import signal
import time
import os

from mct_simulation.find_simulated_camera_images import find_simulated_camera_images


if __name__ == "__main__":
    env = os.environ.copy()
    simulated_camera_images = find_simulated_camera_images()
    proc_list = []
    for simulated_camera_image in simulated_camera_images:
        proc_list.append(subprocess.Popen(['rosrun','image_view','image_view','image:=' + simulated_camera_image,'raw','&'],env=env))

    raw_input('Press Enter to exit.')
    for proc in proc_list:
        proc.send_signal(signal.SIGINT)
