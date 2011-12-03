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
    camera_names = [find_camera_name(simulated_camera_image) for simulated_camera_image in simulated_camera_images]
    proc_list = []
    if len(camera_names) == 2:
        args_str = 'roslaunch camera_pose_calibration calibrate_2_camera.launch camera1_ns:=/sim_camera6/camera camera2_ns:=/sim_camera1/camera checker_rows:=7 checker_cols:=5 checker_size:=0.127'
        proc_list.append(subprocess.Popen(args_str,shell=True,env=env))

    raw_input('Press Enter to exit.')
    for proc in proc_list:
        proc.send_signal(signal.SIGINT)
