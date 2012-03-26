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
from mct_utilities import file_tools

# Start cameras
print(' * starting cameras ... ',end='')
sys.stdout.flush()
camera_master.set_camera_launch_param(frame_rate='camera_calibration',trigger=False)
camera_master.start_cameras()
# Wait until the camera nodes are ready and then start the mjpeg servers
while not mct_introspection.camera_nodes_ready(mode='calibration'):
    time.sleep(0.2)
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

