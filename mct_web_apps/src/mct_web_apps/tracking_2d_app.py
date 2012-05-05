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

from mct_utilities import file_tools
from mct_utilities import redis_tools
from mct_utilities import iface_tools
from mct_camera_tools import mjpeg_servers

DEVELOP = True 
DEBUG = True 

## Setup application w/ sijax
app = flask.Flask(__name__)
app.config["SIJAX_STATIC_PATH"] = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')
app.config["SIJAX_JSON_URI"] = '/static/js/sijax/json2.js'
flask_sijax.Sijax(app)


@app.route('/')
def index():
    """
    Just redirect to the control page.
    """
    return flask.redirect(flask.url_for('show_control'))


@flask_sijax.route(app, '/control')
def show_control(): 
    if flask.g.sijax.is_sijax_request: 
        return
    else:
        ip_iface_ext = redis_tools.get_str(db,'ip_iface_ext')
        tab_list = get_tab_list()
        watchdog_mjpeg_info = get_watchdog_mjpeg_info()
        render_dict = {
                'ip_iface_ext': ip_iface_ext,
                'tab_list': tab_list,
                'watchdog_mjpeg_info': watchdog_mjpeg_info,
                }
        return flask.render_template('tracking_2d_control.html',**render_dict) 

    
@flask_sijax.route(app,'/region/<region>')
def show_region(region):
    if flask.g.sijax.is_sijax_request: 
        return
    else:
        tab_list = get_tab_list()
        mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
        render_dict = {
                'region': region,
                'tab_list': tab_list,
                'mjpeg_info_dict': mjpeg_info_dict,
                }
        return flask.render_template('tracking_2d_region.html', **render_dict) 

@flask_sijax.route(app,'/extra_video/<video>')
def show_extra_video(video):
    if flask.g.sijax.is_sijax_request: 
        return
    else:
        tab_list = get_tab_list()
        mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
        render_dict = {
                'extra_video_name': video,
                'tab_list': tab_list,
                'mjpeg_info_dict': mjpeg_info_dict,
                }
        return flask.render_template('tracking_2d_extra_video.html', **render_dict) 

# ---------------------------------------------------------------------------------
def get_tab_list(): 
    """
    Generates list of tabs and their urls.
    """
    regions_dict = redis_tools.get_dict(db,'regions_dict')
    extra_video_dict = redis_tools.get_dict(db,'extra_video_dict')
    tab_list = [] 
    tab_list.append(('control',flask.url_for('show_control')))
    for region in regions_dict:
        tab_list.append((region, flask.url_for('show_region',region=region)))
    for name in extra_video_dict:
        tab_list.append((name,flask.url_for('show_extra_video', video=name)))
    tab_list.sort()
    return tab_list


def get_watchdog_mjpeg_info():
    """
    Returns a dictionary containing the mjpeg stream information for the
    watchdog information image stream.
    """
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
    watchdog_mjpeg_info = {}
    for v in mjpeg_info_dict.values():
        if v['image_topic'] == '/image_watchdog_info':
            watchdog_mjpeg_info['image_topic'] = v['image_topic']
            watchdog_mjpeg_info['mjpeg_port'] = v['mjpeg_port']
    return watchdog_mjpeg_info

    pass

def setup_redis_db():
    # Create db and add empty camera assignment
    db = redis.Redis('localhost',db=config.redis_db)

    # Add regions dictionary
    regions_dict = file_tools.read_tracking_2d_regions()
    redis_tools.set_dict(db,'regions_dict', regions_dict)

    # Add extra video dictionary
    extra_video_dict = file_tools.read_logging_extra_video()
    redis_tools.set_dict(db,'extra_video_dict',extra_video_dict)

    # Add mjpeg info dictionary
    mjpeg_info_dict = mjpeg_servers.get_mjpeg_info_dict()
    redis_tools.set_dict(db,'mjpeg_info_dict', mjpeg_info_dict)

    # Add external network interface
    machine_def = mct_introspection.get_machine_def()
    ip_iface_ext = iface_tools.get_ip_addr(machine_def['mct_master']['iface-ext'])
    redis_tools.set_str(db,'ip_iface_ext',ip_iface_ext)
    return db

def cleanup():
    pass

# ---------------------------------------------------------------------------------
if __name__ == '__main__':

    db = setup_redis_db()
    atexit.register(cleanup)

    if DEBUG:
        app.debug = True 
        app.run()
    else:
        app.run(host='0.0.0.0',port=5000)
