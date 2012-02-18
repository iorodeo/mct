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
import yaml

import config
import common_args
import mct_introspection
from mct_camera_tools import camera_master
from mct_camera_tools import mjpeg_servers
from mct_camera_calibrator import calibrator_service
from mct_camera_calibrator import calibrator_master

from mct_utilities import redis_tools
from mct_utilities import json_tools
from mct_utilities import iface_tools
from mct_utilities import file_tools

DEVELOP = True 
DEBUG = True 

## Setup application w/ sijax
app = flask.Flask(__name__)
app.config["SIJAX_STATIC_PATH"] = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')
app.config["SIJAX_JSON_URI"] = '/static/js/sijax/json2.js'
flask_sijax.Sijax(app)

# Routes
# ----------------------------------------------------------------------------------

@flask_sijax.route(app, '/')
def index():

    if flask.g.sijax.is_sijax_request:
        flask.g.sijax.register_callback('calibrate_button_onclick', calibrate_button_handler)
        flask.g.sijax.register_callback('save_button_onclick', save_button_handler)
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
        target_info = redis_tools.get_dict(db,'target_info')

        render_dict = {
                'scale'             : scale,
                'scale_options'     : scale_options,
                'image_width'       : image_width,
                'image_height'      : image_height,
                'ip_iface_ext'      : ip_iface_ext,
                'mjpeg_info'        : mjpeg_info,
                'target_info'       : target_info,
                }

        return flask.render_template('camera_calibration.html',**render_dict)

# Sijax request handlers
# ---------------------------------------------------------------------------------

def calibrate_button_handler(obj_response):

    calibrator_list = mct_introspection.get_camera_calibrator_nodes()
    out_list = ['calibrated:']
    for calibrator in calibrator_list:
        good_enough = calibrator_service.good_enough(calibrator)
        if good_enough:
            calibrator_service.calibrate(calibrator)
            camera_name = camera_name_from_calibrator(calibrator)
            out_list.append(camera_name)
    out_str = '<br>'.join(out_list)

    obj_response.html('#develop', out_str)

def save_button_handler(obj_response):

    calibrator_list = mct_introspection.get_camera_calibrator_nodes()
    calibration_dict = {}
    for calibrator in calibrator_list:
        camera_name = camera_name_from_calibrator(calibrator)
        cal_ost_str = calibrator_service.get_calibration(calibrator)
        if cal_ost_str:
            calibration_dict[camera_name] = cal_ost_str

    obj_response.html('#develop', str(calibration_dict))
    write_camera_calibration(calibration_dict)

def reset_button_ok_handler(obj_response):
    calibrator_master.stop()
    target_info = redis_tools.get_dict(db, 'target_info')
    obj_response.html('#develop', str(type(target_info['square'])))
    calibrator_master.start(target_info['chessboard'], target_info['square'])

def reset_button_cancel_handler(obj_response):
    target_info = redis_tools.get_dict(db, 'target_info')
    obj_response.html('#develop', 'reset canceled')

# Utility functions
# ----------------------------------------------------------------------------------

def write_camera_calibration(calibration_dict):
    """
    Writes the camera calibration to file.
    """
    for camera_name, cal_ost_str in calibration_dict.iteritems():
        file_tools.write_camera_calibration(camera_name,cal_ost_str)

def camera_name_from_calibrator(calibrator):
    """
    Returns the name of the camera associated with the given camera calibrator.
    """
    return calibrator.split('/')[2]

def get_image_size(scale): 
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

def cleanup():
    """
    Clean up temporary redis database
    """
    stop_cameras_and_mjpeg_servers()
    db.flushdb()

def setup_redis_db():
    """
    Sets up the redis database for the camera assignemnt application
    """
    # Create db and add empty camera assignment
    db = redis.Redis('localhost',db=config.redis_db)

    mjpeg_info_dict = mjpeg_servers.get_mjpeg_info_dict()
    for camera, info in mjpeg_info_dict.iteritems():
        info['image_topic'] = info['image_topic'].replace('image_raw','image_calibrator')
    redis_tools.set_dict(db,'mjpeg_info_dict', mjpeg_info_dict)

    machine_def = mct_introspection.get_machine_def()
    ip_iface_ext = iface_tools.get_ip_addr(machine_def['mct_master']['iface-ext'])
    redis_tools.set_str(db,'ip_iface_ext',ip_iface_ext)

    scale_default = config.camera_view_table['scale_default']
    redis_tools.set_str(db,'scale', scale_default)

    target_info = file_tools.read_target_info()
    redis_tools.set_dict(db, 'target_info', target_info)

    return db


def start_cameras_and_mjpeg_servers():
    """
    Starts the cameras and mjpeg servers
    """
    if not DEVELOP: 
        # Start cameras
        camera_master.set_camera_launch_param(frame_rate='calibration',trigger=False)
        camera_master.start_cameras()
        # Wait until the camera nodes are ready and then start the mjpeg servers
        while not mct_introspection.camera_nodes_ready(mode='calibration'):
            time.sleep(0.2)
        mjpeg_servers.start_servers()

def stop_cameras_and_mjpeg_servers():
    """
    Stops the cameras and mjpeg servers
    """
    if not DEVELOP:
        mjpeg_servers.stop_servers()
        camera_master.stop_cameras()

# ----------------------------------------------------------------------------------
if __name__ == '__main__':

    start_cameras_and_mjpeg_servers()
    db = setup_redis_db()
    atexit.register(cleanup)

    if DEBUG:
        app.debug = True 
        app.run()
    else:
        app.run(host='0.0.0.0',port=5000)
