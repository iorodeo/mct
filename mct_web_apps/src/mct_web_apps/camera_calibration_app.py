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

import mct_introspection
from mct_camera_tools import camera_master
from mct_camera_tools import mjpeg_servers

from mct_utilities import redis_tools
from mct_utilities import json_tools
from mct_utilities import iface_tools

DEVELOP = False

# Setup application w/ sijax
app = flask.Flask(__name__)
app.config["SIJAX_STATIC_PATH"] = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')
app.config["SIJAX_JSON_URI"] = '/static/js/sijax/json2.js'
flask_sijax.Sijax(app)

# Routes
# ----------------------------------------------------------------------------------

@flask_sijax.route(app, '/')
def index():
    pass
    if flask.g.sijax.is_sijax_request:
        pass
    else:
        return 'Camera Calibration'

# Utility functions
# ----------------------------------------------------------------------------------

def cleanup():
    """
    Clean up temporary redis database
    """
    db.flushdb()

def setup_redis_db():
    """
    Sets up the redis database for the camera assignemnt application
    """
    # Create db and add empty camera assignment
    db = redis.Redis('localhost',db=1)
    return db

def start_cameras_and_mjpeg_servers():
    """
    Starts the cameras and mjpeg servers
    """
    if not DEVELOP: 
        # Start cameras
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

    if 0:
        app.debug = True
        app.run()
    else:
        app.run(host='0.0.0.0',port=5000)
