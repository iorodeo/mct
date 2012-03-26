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
from common_tasks import mjpeg_info_cmp 
from common_tasks import camera_name_cmp

from mct_camera_tools import camera_master
from mct_camera_tools import image_proc_master
from mct_homography import  homography_calibrator_master
from mct_homography import homography_calibrator
from mct_camera_tools import mjpeg_servers
from mct_utilities import redis_tools
from mct_utilities import iface_tools
from mct_utilities import file_tools

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
        flask.g.sijax.register_callback('calibrate_button', calibrate_button_handler)
        flask.g.sijax.register_callback('save_button', save_button_handler)
        flask.g.sijax.register_callback('reset_button_ok', reset_button_ok_handler)
        flask.g.sijax.register_callback('reset_button_cancel', reset_button_cancel_handler)
        flask.g.sijax.register_callback('timer_update', timer_update_handler)
        return flask.g.sijax.process_request()

    else:
        # Get scale and compute image width
        scale, scale_options = common_args.get_scale(config,flask.request)
        redis_tools.set_str(db,'scale', scale)
        image_width, image_height = get_image_size(scale)

        ip_iface_ext = redis_tools.get_str(db,'ip_iface_ext')
        mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
        mjpeg_info = sorted(mjpeg_info_dict.items(), cmp=mjpeg_info_cmp)
        calibration_info = get_calibration_info()
            
        render_dict = {
                'scale'             : scale,
                'scale_options'     : scale_options,
                'image_width'       : image_width,
                'image_height'      : image_height,
                'ip_iface_ext'      : ip_iface_ext,
                'mjpeg_info'        : mjpeg_info,
                'calibration_info'  : calibration_info,
                }

        return flask.render_template('homography_calibration.html',**render_dict)

# sijax request handlers
# ---------------------------------------------------------------------------------------
def calibrate_button_handler(obj_response, camera, topic):
    """
    Handler for requests to run the homography calibration for a given camera
    """
    calibrator_node = get_calibrator_from_topic(topic)
    homography_calibrator.start(calibrator_node)
    obj_response.html('#message', '')
    obj_response.html('#message_table', '')
    obj_response.attr('#message_table', 'style', 'display:none')

def save_button_handler(obj_response):
    """
    Handler for requests to run save the homography matrices
    """
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')

    # Get dictionary of calibration data 
    calibration_dict = {}
    for camera, info in mjpeg_info_dict.iteritems():
        topic = info['image_topic']
        calibrator_node = get_calibrator_from_topic(topic)
        if homography_calibrator.is_calibrated(calibrator_node):
            num_row, num_col, data = homography_calibrator.get_matrix(calibrator_node)
            calibration = {
                    'rows' : num_row,
                    'cols' : num_col,
                    'data' : list(data),
                    }
            calibration_dict[camera] = calibration 

    # Save calibration data 
    file_tools.write_homography_calibration(calibration_dict)

    table_data = []
    camera_list = calibration_dict.keys()
    camera_list.sort(cmp=camera_name_cmp)
    for camera_name in camera_list:
        table_data.append('<tr> <td>')
        table_data.append(camera_name)
        table_data.append('</td> </tr>')
    table_data = '\n'.join(table_data)

    if calibration_dict:
        obj_response.html('#message', 'Saved calibrations for cameras:')
        obj_response.html('#message_table', table_data)
        obj_response.attr('#message_table', 'style', 'display:block')
    else:
        obj_response.html('#message', 'No data to save')
        obj_response.attr('#message_table', 'style', 'display:none')


def reset_button_ok_handler(obj_response):
    """
    Handler for requests to reset the homography calibrators
    """
    obj_response.html('#message', '')
    obj_response.html('#message_table', '')
    obj_response.attr('#message_table', 'style', 'display:none')

    homography_calibrator_master.stop()
    time.sleep(0.5)
    homography_calibrator_master.start()
    while not mct_introspection.homography_calibrator_nodes_ready():
        time.sleep(0.2)

    obj_response.html('#message', 'Homography calibrators reset')
    obj_response.html('#message_table', '')
    obj_response.attr('#message_table', 'style', 'display:none')

def reset_button_cancel_handler(obj_response):
    """
    Handler for canceled request to reset the homography calibrators
    """
    obj_response.html('#message', 'Reset Canceled')
    obj_response.html('#message_table', '')
    obj_response.attr('#message_table', 'style', 'display:none')

def timer_update_handler(obj_response):
    """
    Callback for the timer update function. Resets the modified times for the camera 
    calibration files.
    """
    calibration_info = get_calibration_info()
    for camera, info in calibration_info.iteritems():
        obj_response.html('#{0}_modified_time'.format(camera),info['modified'])

# ---------------------------------------------------------------------------------------
def get_calibrator_from_topic(topic):
    """
    Get name of calibrator node from image topic
    """
    calibrator = [str(x) for x in topic.split('/')]
    calibrator[-1] = 'homography_calibrator' 
    calibrator = '/'.join(calibrator)
    return calibrator

def get_calibration_info():
    """
    Gets the last modified date for any existing homography calibration files.
    """
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
    calibration_info = mct_introspection.get_homography_calibration_info()
    for camera in mjpeg_info_dict:
        if not camera in calibration_info:
            calibration_info[camera] = {'modified': ''}
    return calibration_info

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

        # Wait for rectified images to be ready - required for launching homography
        # calibrators.
        while not mct_introspection.image_rect_ready():
            time.sleep(0.2)

        # Start homography calibrator nodes and wait until ready
        homography_calibrator_master.start()
        while not mct_introspection.homography_calibrator_nodes_ready():
            time.sleep(0.2)

        # Start mjpeg servers and throttleing 
        mjpeg_servers.set_topics(['image_homography_calibration'])
        mjpeg_servers.start_servers()


def kill_nodes():
    if not DEVELOP: 
        print(' * killing nodes ... ',end='')
        homography_calibrator_master.stop()
        image_proc_master.stop_image_proc()
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
