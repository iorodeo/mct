from __future__ import print_function
import roslib
roslib.load_manifest('mct_web_apps')
import rospy
import sys
import time

import mct_introspection
from mct_utilities import file_tools
from mct_utilities import region_tools
from mct_camera_tools import camera_master
from mct_camera_tools import image_proc_master
from mct_camera_tools import mjpeg_servers
from mct_camera_trigger import camera_trigger
from mct_frame_skipper import frame_skipper_master
from mct_image_stitcher import image_stitcher_master
from mct_tracking_2d import three_point_tracker_master 
from mct_tracking_2d import stitched_image_labeler_master
from mct_avi_writer import avi_writer_master
from mct_logging import tracking_pts_logger_master

regions_dict = file_tools.read_tracking_2d_regions()
camera_pairs_dict = file_tools.read_tracking_2d_camera_pairs()
region_tools.check_regions_and_camera_pairs(regions_dict, camera_pairs_dict)

# Stop camera triggers
print(' * stopping camera triggers ... ',end='')
sys.stdout.flush()
camera_trigger.stop()
print('done')

file_tools.rsync_camera_calibrations(verbose=True)

# Start camera nodes and wait until they are ready
print(' * starting camera nodes ... ',end='')
sys.stdout.flush()
camera_master.set_camera_launch_param(
        frame_rate='camera_driver',
        trigger=True
        )
camera_master.start_cameras()
while not mct_introspection.camera_nodes_ready(mode='tracking'):
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
camera_trigger.start(frame_rates['tracking_2d'])
#camera_trigger.start(10)
print('done')

# Start image_proc nodes and wait until they are ready
print(' * starting image proc nodes ... ', end='')
sys.stdout.flush()
image_proc_master.start_image_proc()
while not mct_introspection.image_proc_nodes_ready():
    time.sleep(0.2)
print('done')

# Wait for image rect topics
print(' * waiting for image_rect topics ... ', end='')
sys.stdout.flush()
while not mct_introspection.image_rect_ready():
    time.sleep(0.2)
print('done')

# Start frame skipper nodes and wait unti they are ready
print(' * starting frame skippers ... ', end='')
sys.stdout.flush()
frame_skipper_master.start_frame_skippers()
while not mct_introspection.frame_skippers_ready():
    time.sleep(0.2)
print('done')

# Start image stitcher nodes and wait until stitched image topics ready
print(' * starting image stitchers ... ', end='')
sys.stdout.flush()
image_stitcher_master.start_image_stitchers()
while not mct_introspection.stitched_images_ready():
    time.sleep(0.2)
print('done')

# Start three point tracker nodes and wait until they are ready.
print(' * starting three point trackers ... ', end='')
sys.stdout.flush()
three_point_tracker_master.start_three_point_trackers()
while not mct_introspection.three_point_trackers_ready():
    time.sleep(0.2)
while not mct_introspection.three_point_tracker_synchronizers_ready():
    time.sleep(0.2)
print('done')

# Start stitched image labeler and wait until stitched images are published.
print(' * starting stitched image lablers ... ',end='')
sys.stdout.flush()
stitched_image_labeler_master.start_stitched_image_labelers()
while not mct_introspection.stitched_image_labelers_ready():
    time.sleep(0.2)
print('done')

# Start avi writer nodes 
print(' * starting avi writers ... ', end='')
sys.stdout.flush()
avi_writer_master.start_avi_writers()
print('done')

# Start tracking pts loggers
print(' * starting tracking pts loggers ... ', end='')
sys.stdout.flush()
tracking_pts_logger_master.start_tracking_pts_loggers()
print('done')


# Start mjpeg servers 
print(' * starting mjpeg servers ... ',end='')
sys.stdout.flush()
mjpeg_topics = []
for region in regions_dict:
    mjpeg_topics.append('/{0}/image_stitched_labeled'.format(region))
    mjpeg_topics.append('/{0}/image_tracking_pts'.format(region))
    mjpeg_topics.append('/{0}/image_tracking_info'.format(region))
mjpeg_topics.append('/image_watchdog_info')
mjpeg_servers.set_topics(mjpeg_topics)
mjpeg_servers.start_servers()
print('done')

