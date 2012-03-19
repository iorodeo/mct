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
        flask.g.sijax.register_callback('calibrate_button', calibrate_button_handler)
        flask.g.sijax.register_callback('save_button', save_button_handler)
        flask.g.sijax.register_callback('reset_button_ok', reset_button_ok_handler)
        flask.g.sijax.register_callback('reset_button_cancel', reset_button_cancel_handler)
        #flask.g.sijax.register_callback('timer_update', timer_update_handler)
        return flask.g.sijax.process_request()

    else:
        # Get scale and compute image width
        scale, scale_options = common_args.get_scale(config,flask.request)
        redis_tools.set_str(db,'scale', scale)
        image_width, image_height = get_image_size(scale)

        ip_iface_ext = redis_tools.get_str(db,'ip_iface_ext')
        mjpeg_info = redis_tools.get_dict(db,'mjpeg_info_dict')
        camera_pairs = redis_tools.get_dict(db,'camera_pairs_dict')
        camera_pairs_mjpeg_info = get_camera_pairs_mjpeg_info(camera_pairs, mjpeg_info)
            
        render_dict = {
                'scale': scale,
                'scale_options': scale_options,
                'image_width': image_width,
                'image_height': image_height,
                'ip_iface_ext': ip_iface_ext,
                'camera_pairs': camera_pairs,
                'camera_pairs_mjpeg_info': camera_pairs_mjpeg_info,
                }

        return flask.render_template('transform_2d_calibration.html',**render_dict)

# sijax request handlers
# ---------------------------------------------------------------------------------------

def calibrate_button_handler(obj_response,camera0,camera1):
    calibrator_name = get_calibrator_name(camera0,camera1) 
    transform_2d_calibrator.start(calibrator_name)
    obj_response.html('#message', '')
    obj_response.html('#message_table', '')
    obj_response.attr('#message_table', 'style', 'display:none')

def save_button_handler(obj_response):

    # Collect calibration data for camera pairs
    pairs_dict = redis_tools.get_dict(db,'camera_pairs_dict')
    calibration_dict = {}
    calibration_list = []
    for pairs_list in pairs_dict.values():
        for camera0, camera1 in pairs_list:
            calibrator_name = get_calibrator_name(camera0,camera1)
            if transform_2d_calibrator.is_calibrated(calibrator_name):
                rot, tx, ty = transform_2d_calibrator.get_transform_2d(calibrator_name)
                calibration_dict[(camera0,camera1)] = {
                        'rotation': rot,
                        'translation_x': tx,
                        'translation_y': ty,
                        }
                calibration_list.append((camera0,camera1))

    file_tools.write_transform_2d_calibration(calibration_dict)

    table_data = []
    for camera0, camera1 in calibration_list:
        table_data.append('<tr> <td>')
        table_data.append('{0}, {1}'.format(camera0,camera1))
        table_data.append('</td> </tr>')
    table_data = '\n'.join(table_data)

    if calibration_dict:
        obj_response.html('#message', 'Saved calibrations for camera pairs:')
        obj_response.html('#message_table', table_data)
        obj_response.attr('#message_table', 'style', 'display:block')
    else:
        obj_response.html('#message', 'No data to save')
        obj_response.attr('#message_table', 'style', 'display:none')

def reset_button_ok_handler(obj_response):

    obj_response.html('#message', '')
    obj_response.html('#message_table', '')
    obj_response.attr('#message_table', 'style', 'display:none')

    transform_2d_calibrator_master.stop() 
    time.sleep(0.5) 
    transform_2d_calibrator_master.start()
    #
    #while not mct_introspection.transform_2d_calibrator_nodes_ready():
    #    time.sleep(0.2)

    obj_response.html('#message', '2D transform calibrators reset')
    obj_response.html('#message_table', '')
    obj_response.attr('#message_table', 'style', 'display:none')


def reset_button_cancel_handler(obj_response):
    obj_response.html('#message', 'Reset Canceled')
    obj_response.html('#message_table', '')
    obj_response.attr('#message_table', 'style', 'display:none')

# ---------------------------------------------------------------------------------------

def get_calibrator_name(camera0,camera1):
    return '/{0}_{1}/transform_2d_calibrator'.format(camera0,camera1)

def get_camera_pairs_mjpeg_info(camera_pairs_dict, mjpeg_info_dict):
    camera_pairs_mjpeg_info = {}
    for info_dict in mjpeg_info_dict.values():

        topic = info_dict['image_topic']
        topic_split = topic.split('/')

        # Get camera pair associated with this topic
        camera_pair_str = topic_split[1]
        camera_pair_split = camera_pair_str.split('_')
        camera_0 = '{0}_{1}'.format(camera_pair_split[0],camera_pair_split[1])
        camera_1 = '{0}_{1}'.format(camera_pair_split[2],camera_pair_split[3])

        # Get index of image_transform topic
        image_transform = topic_split[2]
        index = int(image_transform.split('_')[-1])

        camera_pairs_mjpeg_info[(camera_0, camera_1, index)] = info_dict

    return camera_pairs_mjpeg_info 
        

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
                frame_rate='transform_2d_calibration',
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
