#!/usr/bin/python
import subprocess
import signal
import time
import os


if __name__ == "__main__":
    env = os.environ.copy()
    #env['ROS_MASTER_URI'] = 'http://felis:11311'
    if 1:
        proc1 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera1/camera/image_raw','theora','&'],env=env)
        proc2 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera2/camera/image_raw','theora','&'],env=env)
        proc3 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera3/camera/image_raw','theora','&'],env=env)
        proc4 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera4/camera/image_raw','theora','&'],env=env)
        proc5 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera5/camera/image_raw','theora','&'],env=env)
        proc6 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera6/camera/image_raw','theora','&'],env=env)
        proc7 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera7/camera/image_raw','theora','&'],env=env)
        proc8 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera8/camera/image_raw','theora','&'],env=env)
    else:
        proc1 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera1/camera/image_raw','compressed','&'],env=env)
        proc2 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera2/camera/image_raw','compressed','&'],env=env)
        proc3 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera3/camera/image_raw','compressed','&'],env=env)
        proc4 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera4/camera/image_raw','compressed','&'],env=env)
        proc5 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera5/camera/image_raw','compressed','&'],env=env)
        proc6 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera6/camera/image_raw','compressed','&'],env=env)
        proc7 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera7/camera/image_raw','compressed','&'],env=env)
        proc8 = subprocess.Popen(['rosrun','image_view','image_view','image:=/camera8/camera/image_raw','compressed','&'],env=env)

    raw_input('Press Enter to exit.')
    proc1.send_signal(signal.SIGINT)
    proc2.send_signal(signal.SIGINT)
    proc3.send_signal(signal.SIGINT)
    proc4.send_signal(signal.SIGINT)
    proc5.send_signal(signal.SIGINT)
    proc6.send_signal(signal.SIGINT)
    proc7.send_signal(signal.SIGINT)
    proc8.send_signal(signal.SIGINT)
