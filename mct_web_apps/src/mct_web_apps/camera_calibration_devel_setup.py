from __future__ import print_function
import roslib
roslib.load_manifest('mct_web_apps')
import rospy
import time
import sys
import config

import mct_introspection
from mct_camera_tools import camera_master
from mct_camera_tools import mjpeg_servers
from mct_camera_calibrator import calibrator_master
from mct_camera_trigger import camera_trigger
from mct_utilities import file_tools

# Stop camera triggers
print(' * stopping camera triggers ... ',end='')
sys.stdout.flush()
camera_trigger.stop()
print('done')

# Start cameras
print(' * starting cameras ... ',end='')
sys.stdout.flush()
camera_master.set_camera_launch_param(
        frame_rate='camera_driver',
        trigger=True
        )
camera_master.start_cameras()
# Wait until the camera nodes are ready and then start the mjpeg servers
while not mct_introspection.camera_nodes_ready(mode='calibration'):
    time.sleep(0.2)
print('done')

# Delay until all camera nodes are ready and start triggers
print(' * camera nodes sync delay ... ', end='')
sys.stdout.flush()
time.sleep(10)
print('done')

print(' * starting camera triggers ... ', end='')
sys.stdout.flush()
frame_rates = file_tools.read_frame_rates()
camera_trigger.start(frame_rates['camera_calibration'])
print('done')

print(' * starting mjpeg servers... ',end='')
sys.stdout.flush()
mjpeg_servers.set_topics(['image_raw'])
mjpeg_servers.start_servers()
print('done')

print(' * starting camera calibrators ... ',end='')
sys.stdout.flush()
target_info = file_tools.read_target_info(config.camera_calib_target_type)
calibrator_master.start(target_info['size'], target_info['square'])
print('done')

