from __future__ import print_function
import roslib
roslib.load_manifest('mct_web_apps')
import rospy
import mct_introspection
import sys
import time

from mct_camera_tools import camera_master
from mct_camera_tools import image_proc_master
from mct_homography import  homography_calibrator_master
from mct_homography import homography_calibrator
from mct_camera_tools import mjpeg_servers
from mct_utilities import file_tools
from mct_camera_trigger import camera_trigger

# Stop camera triggers
print(' * stopping camera triggers ... ',end='')
sys.stdout.flush()
camera_trigger.stop()
print('done')

# Sync the camera calibration files
file_tools.rsync_camera_calibrations(verbose=True)

# Start camera nodes and wait until they are ready
print(' * starting camera nodes ... ',end='')
camera_master.set_camera_launch_param(
        frame_rate='camera_driver',
        trigger=True
        )
camera_master.start_cameras()
while not mct_introspection.camera_nodes_ready(mode='calibration'):
    time.sleep(0.2)
print('done')

# Delay until all camera nodes are ready 
print(' * camera nodes sync delay ... ', end='')
sys.stdout.flush()
time.sleep(10)
print('done')

print(' * starting camera triggers ... ', end='')
sys.stdout.flush()
frame_rates = file_tools.read_frame_rates()
camera_trigger.start(frame_rates['homography_calibration'])
print('done')

# Start image_proc nodes and wait until they are ready
print(' * starting image proc nodes ... ', end='')
sys.stdout.flush()
image_proc_master.start_image_proc()
while not mct_introspection.image_proc_nodes_ready():
    time.sleep(0.2)
print('done')

# Wait for rectified images to be ready - required for launching homography
# calibrators.
print(' * waiting for image rect topics ...', end='')
sys.stdout.flush()
while not mct_introspection.image_rect_ready():
    time.sleep(0.2)
print('done')

# Start homography calibrator nodes and wait until ready
print(' * starting homography calibrators ... ', end='')
sys.stdout.flush()
homography_calibrator_master.start()
while not mct_introspection.homography_calibrator_nodes_ready():
    time.sleep(0.2)
print('done')

# Start mjpeg servers and throttleing 
print(' * starting mjpeg servers ... ',end='')
sys.stdout.flush()
mjpeg_servers.set_topics(['image_homography_calibration'])
mjpeg_servers.start_servers()
print('done')
