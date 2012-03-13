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

DEVELOP = False 
DEBUG = False 
TARGET_TYPE = 'chessboard'

## Setup application w/ sijax
app = flask.Flask(__name__)
app.config["SIJAX_STATIC_PATH"] = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')
app.config["SIJAX_JSON_URI"] = '/static/js/sijax/json2.js'
flask_sijax.Sijax(app)

@flask_sijax.route(app, '/')
def index():

    if flask.g.sijax.is_sijax_request:
        pass
    else:
        return "Hello"

def setup_redis_db():
    print('* setting up redis db')

def start_nodes():

    if not DEVELOP: 

        # Start camera nodes and wait until they are ready
        print('* starting camera nodes ... ',end='')
        camera_master.set_camera_launch_param(
                frame_rate='homography_calibration',
                trigger=False
                )
        camera_master.start_cameras()
        while not mct_introspection.camera_nodes_ready(mode='calibration'):
            time.sleep(0.2)
        print('done')

        # Start image_proc nodes and wait until they are ready
        print('* starting image proc nodes ... ', end='')
        image_proc_master.start_image_proc()
        while not mct_introspection.image_proc_nodes_ready():
            time.sleep(0.2)
        print('done')

        # Wait for rectified images to be ready - required for launching homography
        # calibrators.
        print('* waiting for image rect topics ...', end='')
        while not mct_introspection.image_rect_ready():
            time.sleep(0.2)
        print('done')

        # Start homography calibrator nodes and wait until ready
        print('* starting homography calibrators', end='')
        value = homography_calibrator_master.start()
        while not mct_introspection.homography_calibrator_nodes_ready():
            time.sleep(0.2)
        print('done')

        # Start mjpeg servers and throttleing 


def kill_nodes():
    print('* killing nodes ... ',end='')
    homography_calibrator_master.stop()
    image_proc_master.stop_image_proc()
    camera_master.stop_cameras()
    print('done')

def cleanup():
    kill_nodes()

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
