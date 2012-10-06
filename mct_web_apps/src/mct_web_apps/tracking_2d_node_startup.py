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
from mct_frame_drop_corrector import frame_drop_corrector_master
from mct_frame_drop_corrector import frame_drop_corrector
from mct_image_stitcher import image_stitcher_master
from mct_tracking_2d import three_point_tracker_master 
from mct_tracking_2d import stitched_image_labeler_master
from mct_avi_writer import avi_writer_master
from mct_logging import tracking_pts_logger_master

# Startup parameters
ready_poll_dt = 0.2      # Sleep time for polling loop when waiting for node or topic to start
camera_startup_t = 10.0  # Sleep interval for waiting for camera processes to startup 
buffer_flush_t = 2.0     # Sleep interval for flushing buffers

# Get Startup configuration information
regions_dict = file_tools.read_tracking_2d_regions()
extra_video_dict = file_tools.read_logging_extra_video()
camera_pairs_dict = file_tools.read_tracking_2d_camera_pairs()
region_tools.check_regions_and_camera_pairs(regions_dict, camera_pairs_dict)

def debug_print(*args,**kwarg):
    debug = kwarg.pop('debug')
    if debug:
        print(*args,**kwarg)

def tracking_2d_node_startup(debug=False):

    # Stop camera triggers
    debug_print(' * stopping camera triggers ... ',end='',debug=debug)
    sys.stdout.flush()
    camera_trigger.stop()
    debug_print('done',debug=debug)
    
    file_tools.rsync_camera_calibrations(verbose=True)
    
    # Start camera nodes and wait until they are ready
    debug_print(' * starting camera nodes ... ',end='',debug=debug)
    sys.stdout.flush()
    camera_master.set_camera_launch_param(
            frame_rate='camera_driver',
            trigger=True
            )
    camera_master.start_cameras()
    while not mct_introspection.camera_nodes_ready(mode='tracking'):
        time.sleep(ready_poll_dt)
    debug_print('done',debug=debug)
    
    # Delay until all camera nodes are ready 
    debug_print(' * camera nodes sync delay ... ', end='',debug=debug)
    sys.stdout.flush()
    time.sleep(camera_startup_t)
    debug_print('done',debug=debug)
    
    # Restart camera triggers
    debug_print(' * starting camera triggers ... ', end='',debug=debug)
    sys.stdout.flush()
    frame_rates = file_tools.read_frame_rates()
    camera_trigger.start(frame_rates['tracking_2d'])
    debug_print('done',debug=debug)
    
    # Start image_proc nodes and wait until they are ready
    debug_print(' * starting image proc nodes ... ', end='',debug=debug)
    sys.stdout.flush()
    image_proc_master.start_image_proc()
    while not mct_introspection.image_proc_nodes_ready():
        time.sleep(ready_poll_dt)
    debug_print('done',debug=debug)
    
    # Wait for image rect topics
    debug_print(' * waiting for image_rect topics ... ', end='',debug=debug)
    sys.stdout.flush()
    while not mct_introspection.image_rect_ready():
        time.sleep(ready_poll_dt)
    debug_print('done',debug=debug)

    # Start frame drop corrector nodes and wait until they are ready
    debug_print(' * starting frame drop correctors ... ', end='', debug=debug)
    sys.stdout.flush()
    frame_drop_corrector_master.start()
    while not mct_introspection.frame_drop_correctors_ready():
        time.sleep(ready_poll_dt)
    debug_print('done',debug=debug)

    # Stop camera triggers again to sync frame drop correctors.  Note, It
    # appears we do need to stop the camera triggers twice during startup. 
    # Sync'ing the frame drop correctors requires stopping the camera triggers
    # resetting the frame drop correctors and then re-starting the triggers.
    debug_print(' * stopping camera triggers ... ',end='',debug=debug)
    sys.stdout.flush()
    camera_trigger.stop()
    debug_print('done',debug=debug)

    # Reset frame drop correctors 
    debug_print(' * frame drop corrector sync delay ... ',end='',debug=debug)
    time.sleep(buffer_flush_t)
    frame_drop_corrector.reset_all()
    debug_print('done',debug=debug)

    # --------------------------------------------------------------------
    # Need to reset time stamp watchdog ... or change how it functions 
    # 
    # Delay working on this until we know what modifcations we are making
    # to the watchdog.
    # --------------------------------------------------------------------

    # Restart camera triggers
    debug_print(' * starting camera triggers ... ', end='',debug=debug)
    sys.stdout.flush()
    frame_rates = file_tools.read_frame_rates()
    camera_trigger.start(frame_rates['tracking_2d'])
    debug_print('done',debug=debug)
    
    # Start frame skipper nodes and wait unti they are ready
    debug_print(' * starting frame skippers ... ', end='',debug=debug)
    sys.stdout.flush()
    frame_skipper_master.start_frame_skippers()
    while not mct_introspection.frame_skippers_ready():
        time.sleep(ready_poll_dt)
    debug_print('done',debug=debug)
    
    # Start image stitcher nodes and wait until stitched image topics ready
    debug_print(' * starting image stitchers ... ', end='',debug=debug)
    sys.stdout.flush()
    image_stitcher_master.start_image_stitchers()
    while not mct_introspection.stitched_images_ready():
        time.sleep(ready_poll_dt)
    debug_print('done',debug=debug)
    
    # Start three point tracker nodes and wait until they are ready.
    debug_print(' * starting three point trackers ... ', end='',debug=debug)
    sys.stdout.flush()
    three_point_tracker_master.start_three_point_trackers()
    while not mct_introspection.three_point_trackers_ready():
        time.sleep(ready_poll_dt)
    while not mct_introspection.three_point_tracker_synchronizers_ready():
        time.sleep(ready_poll_dt)
    debug_print('done',debug=debug)
    
    # Start stitched image labeler and wait until stitched images are published.
    debug_print(' * starting stitched image lablers ... ',end='',debug=debug)
    sys.stdout.flush()
    stitched_image_labeler_master.start_stitched_image_labelers()
    while not mct_introspection.stitched_image_labelers_ready():
        time.sleep(ready_poll_dt)
    debug_print('done',debug=debug)
    
    # Start avi writer nodes 
    debug_print(' * starting avi writers ... ', end='',debug=debug)
    sys.stdout.flush()
    avi_writer_master.start_avi_writers()
    debug_print('done',debug=debug)
    
    # Start tracking pts loggers
    debug_print(' * starting tracking pts loggers ... ', end='',debug=debug)
    sys.stdout.flush()
    tracking_pts_logger_master.start_tracking_pts_loggers()
    debug_print('done',debug=debug)
    
    # Start mjpeg servers 
    debug_print(' * starting mjpeg servers ... ',end='',debug=debug)
    sys.stdout.flush()
    
    # Set topics for mjpeg servers
    mjpeg_topics = []
    # Add time stamp watchdog image
    mjpeg_topics.append('/image_frame_drop_watchdog')
    
    # Add the stitched and tracking pts images for all regions
    for region in regions_dict:
        mjpeg_topics.append('/{0}/image_stitched_labeled'.format(region))
        mjpeg_topics.append('/{0}/image_tracking_pts'.format(region))
        mjpeg_topics.append('/{0}/image_tracking_info'.format(region))
    
    # Add any extra video images
    for v in extra_video_dict.values():
        mjpeg_topics.append(v)
    
    mjpeg_servers.set_topics(mjpeg_topics)
    mjpeg_servers.start_servers()
    debug_print('done',debug=debug)

# ---------------------------------------------------------------------------------
if __name__ == '__main__':

    tracking_2d_node_startup(debug=True)

