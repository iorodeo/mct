#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_web_apps')
import rospy
import os
import os.path
import sys
import flask
import flask_sijax
import redis
import atexit
import time
import config
import common_args
import mct_introspection

from common_tasks import get_image_size

from mct_camera_tools import camera_master
from mct_camera_tools import image_proc_master
from mct_transform_2d import transform_2d_calibrator_master
from mct_transform_2d import transform_2d_calibrator
from mct_camera_tools import mjpeg_servers
from mct_utilities import redis_tools
from mct_utilities import iface_tools
from mct_utilities import file_tools
from mct_utilities import region_tools

DEVELOP = True 
DEBUG = True 

## Setup application w/ sijax
app = flask.Flask(__name__)
app.config["SIJAX_STATIC_PATH"] = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')
app.config["SIJAX_JSON_URI"] = '/static/js/sijax/json2.js'
flask_sijax.Sijax(app)

@flask_sijax.route(app, '/')
def index():

    if flask.g.sijax.is_sijax_request: 
        return flask.g.sijax.process_request()

    else:
        # Get scale and compute image width
        scale, scale_options = common_args.get_scale(config,flask.request)
        redis_tools.set_str(db,'scale', scale)
        image_width, image_height = get_image_size(scale)

        ip_iface_ext = redis_tools.get_str(db,'ip_iface_ext')
        mjpeg_info = redis_tools.get_dict(db,'mjpeg_info_dict')
        camera_pairs = redis_tools.get_dict(db,'camera_pairs_dict')
        pairs_to_mjpeg_info = get_pairs_to_mjpeg_info(camera_pairs, mjpeg_info)
            
        render_dict = {
                'scale'             : scale,
                'scale_options'     : scale_options,
                'image_width'       : image_width,
                'image_height'      : image_height,
                'ip_iface_ext'      : ip_iface_ext,
                'mjpeg_info'        : mjpeg_info,
                'camera_pairs'      : camera_pairs,
                'pairs_to_mjpeg_info' : pairs_to_mjpeg_info,
                'develop'            : str(pairs_to_mjpeg_info),
                }

        return flask.render_template('transform_2d_calibration.html',**render_dict)

# sijax request handlers
# ---------------------------------------------------------------------------------------

def get_pairs_to_mjpeg_info(camera_pairs_dict, mjpeg_info_dict):
    camera_pair_info_dict = {}
    for pairs_list in camera_pairs_dict.values():
        for cam0, cam1 in pairs_list:
            for info in mjpeg_info_dict.values():
                cam_pair_str = '{0}_{1}'.format(cam0, cam1)
                if cam_pair_str in info['image_topic'].split('/'): 
                    camera_pair_info_dict[(cam0, cam1)] = info
    return camera_pair_info_dict
        

def setup_redis_db():
    # Create db and add empty camera assignment
    db = redis.Redis('localhost',db=config.redis_db)

    # Add mjpeg information
    mjpeg_info_dict = mjpeg_servers.get_mjpeg_info_dict()
    redis_tools.set_dict(db,'mjpeg_info_dict', mjpeg_info_dict)

    # Add camera pairs information
    camera_pairs_dict = file_tools.read_tracking_2d_camera_pairs()
    redis_tools.set_dict(db,'camera_pairs_dict', camera_pairs_dict)

    # Add external network interface
    machine_def = mct_introspection.get_machine_def()
    ip_iface_ext = iface_tools.get_ip_addr(machine_def['mct_master']['iface-ext'])
    redis_tools.set_str(db,'ip_iface_ext',ip_iface_ext)

    # Add default scale for image view
    scale_default = config.camera_view_table['scale_default']
    redis_tools.set_str(db,'scale', scale_default)

    return db

def start_nodes():
    if not DEVELOP: 

        file_tools.rsync_camera_calibrations()
        
        # Start camera nodes and wait until they are ready
        camera_master.set_camera_launch_param(
                frame_rate='homography_calibration',
                trigger=False
                )
        camera_master.start_cameras()
        while not mct_introspection.camera_nodes_ready(mode='calibration'):
            time.sleep(0.2)
        
        # Start image_proc nodes and wait until they are ready
        image_proc_master.start_image_proc()
        while not mct_introspection.image_proc_nodes_ready():
            time.sleep(0.2)
        
        # Wait for rectified images to be ready - required for launching transform 
        # calibrators.
        while not mct_introspection.image_rect_ready():
            time.sleep(0.2)
        
        # Start transform 2d calibrator nodes and wait until ready
        transform_2d_calibrator_master.start()
        while not mct_introspection.transform_2d_calibrator_nodes_ready():
            time.sleep(0.2)
        
        # Start mjpeg servers and throttleing 
        topic_list = [ 'image_transform_calibration_0', 'image_transform_calibration_1']
        mjpeg_servers.set_topics(topic_list)
        mjpeg_servers.start_servers()

def check_regions_and_pairs():
    """
    Make sure the tracking regions and camera pairs are well defined.
    """
    regions_dict = file_tools.read_tracking_2d_regions()
    camera_pairs_dict = file_tools.read_tracking_2d_camera_pairs()
    region_tools.check_regions_and_camera_pairs(regions_dict, camera_pairs_dict)



def cleanup():
    db.flushdb()

# ---------------------------------------------------------------------------------
if __name__ == '__main__':

    check_regions_and_pairs()
    start_nodes()
    db = setup_redis_db()
    atexit.register(cleanup)

    if DEBUG:
        app.debug = True 
        app.run()
    else:
        app.run(host='0.0.0.0',port=5000)
