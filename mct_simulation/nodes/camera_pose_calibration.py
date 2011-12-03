#!/usr/bin/python
from __future__ import division
import roslib
roslib.load_manifest('mct_simulation')
import rospy
import subprocess
import signal
import time
import os

from mct_simulation.find_simulated_camera_images import find_simulated_camera_images, find_camera_name


if __name__ == "__main__":
    env = os.environ.copy()
    simulated_camera_images = find_simulated_camera_images()
    proc_list = []
    for simulated_camera_image in simulated_camera_images:
        camera_name = find_camera_name(simulated_camera_image)
        print camera_name
        # proc_list.append(subprocess.Popen(['rosrun','camera_calibration','cameracalibrator.py','--size 7x5','--square 5.0','image:=' + simulated_camera_image,'camera:=' + camera_name,'&'],env=env))
        # args_str = 'rosrun camera_calibration cameracalibrator.py --size 7x5 --square 5.0 image:=' + simulated_camera_image + ' camera:=' + camera_name + ' &'
        # proc_list.append(subprocess.Popen(args_str,shell=True,env=env))

    raw_input('Press Enter to exit.')
    for proc in proc_list:
        proc.send_signal(signal.SIGINT)
