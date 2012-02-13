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
from mct_camera_calibrator import calibrator_master

from mct_utilities import redis_tools
from mct_utilities import json_tools
from mct_utilities import iface_tools

DEVELOP = True 
DEBUG = False 

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
        flask.g.sijax.register_callback('radio_button_onclick', radio_button_onclick_handler)
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

        return flask.render_template('camera_calibration.html',**render_dict)

# Sijax request handlers
# ---------------------------------------------------------------------------------

def radio_button_onclick_handler(obj_response, form_values):
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
    ip_iface_ext = redis_tools.get_str(db,'ip_iface_ext')
    scale = redis_tools.get_str(db,'scale')

    cal_camera = str(form_values['selected_camera'])
    cal_image_topic = mjpeg_info_dict[cal_camera]['camera_topic']
    cal_camera_topic = '/'.join(cal_image_topic.split('/')[:-1])
    cal_mjpeg_port = mjpeg_info_dict[cal_camera]['mjpeg_port'] + 1000

    # Stop currently running image calibrator and start new version on selected camera.
    calibrator_master.stop()
    calibrator_master.start(cal_camera_topic, cal_image_topic, '8x6','0.0254')

    # Get img src for camera calibration
    image_width, image_height = get_image_size(scale)
    mjpeg_options = '?width={0}?height={1}?quality={2}'.format(image_width,image_height,60)

    for camera_name, camera_info in mjpeg_info_dict.iteritems():
        mjpeg_port = camera_info['mjpeg_port']
        if camera_name == cal_camera:
            img_src = 'http://{0}:{1}/stream?topic=/image_calibration{2}'.format(ip_iface_ext, mjpeg_port, mjpeg_options)
            temp = img_src
        else:
            image_raw = camera_info['camera_topic']
            img_src = 'http://{0}:{1}/stream?topic={2}_throttle{3}'.format(ip_iface_ext, mjpeg_port, image_raw, mjpeg_options)
        obj_response.attr('#{0}'.format(camera_name),'src', img_src)

    obj_response.html('#develop','image: {0} <br>camera: {1} <br>scale: {2} <br>cal mjpeg_port: {3} <br>temp: {4}'.format(cal_image_topic, cal_camera_topic, scale, cal_mjpeg_port, temp))


# Utility functions
# ----------------------------------------------------------------------------------

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
    redis_tools.set_dict(db,'mjpeg_info_dict', mjpeg_info_dict)

    machine_def = mct_introspection.get_machine_def()
    ip_iface_ext = iface_tools.get_ip_addr(machine_def['mct_master']['iface-ext'])
    redis_tools.set_str(db,'ip_iface_ext',ip_iface_ext)

    scale_default = config.camera_view_table['scale_default']
    redis_tools.set_str(db,'scale', scale_default)

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
