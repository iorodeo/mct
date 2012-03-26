from __future__ import print_function
import roslib
roslib.load_manifest('mct_web_apps')
import rospy
import mct_introspection
import time
import sys

import mct_active_target
from mct_camera_tools import camera_master
from mct_zoom_tool import zoom_tool_master
from mct_camera_tools import mjpeg_servers
from mct_camera_trigger import camera_trigger

# Stop camera triggers
print(' * stopping camera triggers ... ',end='')
sys.stdout.flush()
camera_trigger.stop()
print('done')

# Start camera nodes and wait until they are ready
print(' * starting camera nodes ... ',end='')
sys.stdout.flush()
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
camera_trigger.start(frame_rates['zoom_calibration'])
print('done')

# Start zoom tool nodes and wait until they are ready
print(' * starting zoom tool nodes ... ',end='')
sys.stdout.flush()
zoom_tool_master.start()
while not mct_introspection.zoom_tool_image_ready():
    time.sleep(0.2)
print('done')

# Start mjpeg servers and throttleing 
print(' * starting mjpeg servers ... ',end='')
sys.stdout.flush()
mjpeg_servers.set_topics(['image_zoom_tool'])
mjpeg_servers.start_servers()
print('done')

# Turn on active target two led patter
print(' * turn of led pair ... ',end='')
sys.stdout.flush()
mct_active_target.led_pair()
print('done')
