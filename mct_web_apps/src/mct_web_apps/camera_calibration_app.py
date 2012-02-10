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

from mct_utilities import redis_tools
from mct_utilities import json_tools
from mct_utilities import iface_tools

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
        pass
    else:
        # Get scale and compute image width
        scale, scale_options = common_args.get_scale(config,flask.request)
        image_width = int(config.camera_image['width']*float(scale))
        image_height = int(config.camera_image['height']*float(scale))

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

        return flask.render_template('camera_calibration.html',**render_dict)

# Utility functions
# ----------------------------------------------------------------------------------

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
    redis_tools.set_dict(db,'mjpeg_info_dict', mjpeg_info_dict)

    machine_def = mct_introspection.get_machine_def()
    ip_iface_ext = iface_tools.get_ip_addr(machine_def['mct_master']['iface-ext'])
    redis_tools.set_str(db,'ip_iface_ext',ip_iface_ext)

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
