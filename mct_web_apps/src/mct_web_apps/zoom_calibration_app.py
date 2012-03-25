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
import mct_active_target

from common_tasks import get_image_size
from common_tasks import mjpeg_info_cmp 
from common_tasks import camera_name_cmp

from mct_camera_tools import camera_master
from mct_zoom_tool import  zoom_tool_master
from mct_camera_tools import mjpeg_servers
from mct_utilities import redis_tools
from mct_utilities import iface_tools
from mct_utilities import file_tools

from single_camera_view_blueprint import single_camera_view

DEBUG = False 
DEVELOP = False 

## Setup application w/ sijax
app = flask.Flask(__name__)
app.config["SIJAX_STATIC_PATH"] = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')
app.config["SIJAX_JSON_URI"] = '/static/js/sijax/json2.js'
flask_sijax.Sijax(app)

app.register_blueprint(single_camera_view)

@app.route('/')
def index(): 

    # Get scale and compute image width
    scale, scale_options = common_args.get_scale(config,flask.request)
    redis_tools.set_str(db,'scale', scale)
    image_width, image_height = get_image_size(scale)

    ip_iface_ext = redis_tools.get_str(db,'ip_iface_ext')
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
    mjpeg_info = sorted(mjpeg_info_dict.items(), cmp=mjpeg_info_cmp)

    # Build dict of urls for single camera views
    single_view_url = {}
    for camera in mjpeg_info_dict:
        single_view_url[camera] = flask.url_for('single_camera_view.page',camera=camera)
        
    render_dict = {
            'scale'             : scale,
            'scale_options'     : scale_options,
            'image_width'       : image_width,
            'image_height'      : image_height,
            'ip_iface_ext'      : ip_iface_ext,
            'mjpeg_info'        : mjpeg_info,
            'single_view_url'   : single_view_url,
            }

    return flask.render_template('zoom_calibration.html',**render_dict)


def setup_redis_db():
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
                frame_rate='zoom_calibration',
                trigger=False
                )
        camera_master.start_cameras()
        while not mct_introspection.camera_nodes_ready(mode='calibration'):
            time.sleep(0.2)
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


def kill_nodes():
    if not DEVELOP: 
        print(' * killing nodes ... ',end='')
        mct_active_target.off()
        mjpeg_servers.stop_servers()
        zoom_tool_master.stop()
        camera_master.stop_cameras()
        print('done')

def cleanup():
    #kill_nodes()
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
