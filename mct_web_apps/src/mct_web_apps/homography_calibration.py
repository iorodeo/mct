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

from mct_camera_tools import camera_master
from mct_camera_tools import image_proc_master
from mct_homography import  homography_calibrator_master
from mct_homography import homography_calibrator
from mct_camera_tools import mjpeg_servers
from mct_utilities import redis_tools
from mct_utilities import json_tools
from mct_utilities import iface_tools
from mct_utilities import file_tools

DEVELOP = True 
DEBUG = True 
TARGET_TYPE = 'chessboard'

## Setup application w/ sijax
app = flask.Flask(__name__)
app.config["SIJAX_STATIC_PATH"] = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')
app.config["SIJAX_JSON_URI"] = '/static/js/sijax/json2.js'
flask_sijax.Sijax(app)

@flask_sijax.route(app, '/')
def index():

    if flask.g.sijax.is_sijax_request: 
        flask.g.sijax.register_callback('calibrate_button', calibrate_button_handler)
        flask.g.sijax.register_callback('save_button', save_button_handler)
        flask.g.sijax.register_callback('reset_button_ok', reset_button_ok_handler)
        flask.g.sijax.register_callback('reset_button_cancel', reset_button_cancel_handler)
        return flask.g.sijax.process_request()

    else:
        # Get scale and compute image width
        scale, scale_options = common_args.get_scale(config,flask.request)
        redis_tools.set_str(db,'scale', scale)
        image_width, image_height = get_image_size(scale)

        ip_iface_ext = redis_tools.get_str(db,'ip_iface_ext')
        mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
        mjpeg_info = sorted(mjpeg_info_dict.items(), cmp=mjpeg_info_cmp)
            
        render_dict = {
                'scale'             : scale,
                'scale_options'     : scale_options,
                'image_width'       : image_width,
                'image_height'      : image_height,
                'ip_iface_ext'      : ip_iface_ext,
                'mjpeg_info'        : mjpeg_info,
                }

        return flask.render_template('homography_calibration.html',**render_dict)

        return "Hello"

# sijax request handlers
# ---------------------------------------------------------------------------------------
def calibrate_button_handler(obj_response, camera, topic):
    calibrator_node = get_calibrator_from_topic(topic)
    homography_calibrator.start(calibrator_node)
    obj_response.html('#develop', str(calibrator_node))

def save_button_handler(obj_response):
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')

    # Get dictionary of calibration data 
    calibration_dict = {}
    for camera, info in mjpeg_info_dict.iteritems():
        topic = info['image_topic']
        calibrator_node = get_calibrator_from_topic(topic)
        if homography_calibrator.is_calibrated(calibrator_node):
            num_row, num_col, data = homography_calibrator.get_matrix(calibrator_node)
            calibration = {
                    'num_row': num_row,
                    'num_col': num_col,
                    'data'   : data,
                    }
            calibration_dict[camera] = calibration 

    # Save calibration data 

    obj_response.html('#develop', str(calibration_dict))
    #obj_response.html('#develop', 'save button')

def reset_button_ok_handler(obj_response):
    obj_response.html('#develop', 'reset button ok')

def reset_button_cancel_handler(obj_response):
    obj_response.html('#develop', 'reset button cancel')

# ---------------------------------------------------------------------------------------
def get_calibrator_from_topic(topic):
    """
    Get name of calibrator node from image topic
    """
    calibrator = [str(x) for x in topic.split('/')]
    calibrator[-1] = 'homography_calibrator' 
    calibrator = '/'.join(calibrator)
    return calibrator

def get_image_size(scale): 
    """
    Get size of image from scale
    """
    image_width = int(config.camera_image['width']*float(scale))
    image_height = int(config.camera_image['height']*float(scale))
    return image_width, image_height

def mjpeg_info_cmp(x,y):
    """
    Comparison function for sorting a list of (camera_name, camera_info) pairs.
    """
    name_x = x[0]
    name_y = y[0]
    value_x = int(name_x.replace('camera_', ''))
    value_y = int(name_y.replace('camera_', ''))
    if value_x > value_y:
        return 1
    elif value_y > value_x:
        return -1
    else:
        return 0

def setup_redis_db():
    print(' * setting up redis db')

    # Create db and add empty camera assignment
    db = redis.Redis('localhost',db=config.redis_db)

    # Add mjpeg info dictionary
    mjpeg_info_dict = mjpeg_servers.get_mjpeg_info_dict()
    redis_tools.set_dict(db,'mjpeg_info_dict', mjpeg_info_dict)

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

        # Start camera nodes and wait until they are ready
        print(' * starting camera nodes ... ',end='')
        sys.stdout.flush()
        camera_master.set_camera_launch_param(
                frame_rate='homography_calibration',
                trigger=False
                )
        camera_master.start_cameras()
        while not mct_introspection.camera_nodes_ready(mode='calibration'):
            time.sleep(0.2)
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
        mjpeg_servers.set_transport('image_homography_calibration')
        mjpeg_servers.start_servers()
        print('done')


def kill_nodes():
    if not DEVELOP: 
        print(' * killing nodes ... ',end='')
        homography_calibrator_master.stop()
        image_proc_master.stop_image_proc()
        camera_master.stop_cameras()
        print('done')

def cleanup():
    kill_nodes()
    db.flushdb()

# ---------------------------------------------------------------------------------
if __name__ == '__main__':

    start_nodes()
    db = setup_redis_db()
    atexit.register(cleanup)

    if DEBUG:
        app.debug = True 
        app.run()
    else:
        app.run(host='0.0.0.0',port=5000)
