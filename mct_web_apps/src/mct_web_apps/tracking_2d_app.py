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
import datetime
import config
import common_args
import mct_introspection

from common_tasks import camera_name_cmp
from mct_utilities import file_tools
from mct_utilities import redis_tools
from mct_utilities import iface_tools
from mct_camera_tools import mjpeg_servers
from mct_camera_trigger import camera_trigger
from mct_watchdog import frame_drop_watchdog
from mct_logging import tracking_pts_logger
from mct_avi_writer import avi_writer
from mct_light_control import led_control
from tracking_2d_node_startup import tracking_2d_node_startup
from mct_tracking_2d import three_point_tracker_synchronizer
from mct_frame_drop_corrector import frame_drop_corrector
from mct_image_stitcher import image_stitcher
from mct_rand_sync import reset_rand_sync

DEVELOP = False 
DEBUG = False 

OPERATING_MODES = ('standby', 'preview', 'recording')

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

# Routes
# -----------------------------------------------------------------------------

@flask_sijax.route(app, '/control')
def show_control(): 
    """
    Tracking program main control page. Enables users to start/stop recording and 
    displays information about the running system.
    """
    if flask.g.sijax.is_sijax_request: 
        flask.g.sijax.register_callback('mode_change_request', mode_change_handler)
        flask.g.sijax.register_callback('timer_update', timer_update_handler)
        return flask.g.sijax.process_request()
    else:
        current_mode = redis_tools.get_str(db,'current_mode')
        watchdog_mjpeg_info = get_watchdog_mjpeg_info()
        regions_dict = redis_tools.get_dict(db,'regions_dict')
        extra_video_dict = redis_tools.get_dict(db,'extra_video_dict')
        homography_cal_info = get_homography_calibration_info()
        transform_2d_cal_info = get_transform_2d_calibration_info()
        camera_cal_info = get_camera_calibration_info()
        logging_params_dict = redis_tools.get_dict(db,'logging_params_dict')
        camera_assignment = get_camera_assignment()
        all_cameras_sorted = camera_assignment.keys()
        all_cameras_sorted.sort(cmp=camera_name_cmp)
        camera_pairs_dict = get_camera_pairs_dict()

        render_dict = get_base_render_dict()
        page_render_dict = {
                'operating_modes': OPERATING_MODES,
                'current_mode': current_mode,
                'watchdog_mjpeg_info': watchdog_mjpeg_info,
                'regions_dict': regions_dict,
                'extra_video_dict': extra_video_dict,
                'logging_params_dict': logging_params_dict,
                'homography_cal_info': homography_cal_info,
                'transform_2d_cal_info': transform_2d_cal_info,
                'camera_cal_info': camera_cal_info,
                'camera_assignment': camera_assignment,
                'all_cameras_sorted': all_cameras_sorted,
                'camera_pairs_dict': camera_pairs_dict,
                }
        render_dict.update(page_render_dict)
        return flask.render_template('tracking_2d_control.html',**render_dict) 

@app.route('/region/<region>')
def show_region(region):
    """
    Handles requests to view the images and information stream for the given tracking
    region.
    """
    render_dict = get_base_render_dict()
    region_mjpeg_info = get_region_mjpeg_info(region)
    page_render_dict = {
            'region': region,
            'region_mjpeg_info': region_mjpeg_info,
            }
    render_dict.update(page_render_dict)
    return flask.render_template('tracking_2d_region.html', **render_dict) 

@app.route('/extra_video/<video>')
def show_extra_video(video):
    """
    Handles requests to view the requested extra video streams.
    """
    ip_iface_ext = redis_tools.get_str(db,'ip_iface_ext')
    tab_list = get_tab_list()
    extra_video_mjpeg_info = get_extra_video_mjpeg_info(video)
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
    render_dict = get_base_render_dict()
    page_render_dict = {
            'extra_video_name': video,
            'extra_video_mjpeg_info': extra_video_mjpeg_info,
            }
    render_dict.update(page_render_dict)
    return flask.render_template('tracking_2d_extra_video.html', **render_dict) 

@app.route('/ros_info')
def show_ros_info():
    render_dict = get_base_render_dict()
    ros_nodes_list = mct_introspection.get_nodes()
    ros_topics_list = mct_introspection.get_topics()
    ros_services_list = mct_introspection.get_services()
    ros_info_list = [
            ('nodes',  ros_nodes_list), 
            ('topics', ros_topics_list),
            ('services', ros_services_list),
            ]
    page_render_dict = {
            'ros_info_list': ros_info_list,
            }
    render_dict.update(page_render_dict)
    return flask.render_template('tracking_2d_ros_info.html', **render_dict)

@app.route('/lighting',methods=['GET','POST'])
def lighting():
    render_dict = get_base_render_dict()
    lighting_values = get_empty_lighting_values()
    have_enable = False
    have_current = False
    channel = ''
    enable = False
    iset = 0

    if flask.request.method == 'POST':
        for form_key in flask.request.form:
            form_key_split = str(form_key).split('?')
            form_key_end = form_key_split[-1]
            if form_key_end in ('get', 'set'):
                name = str(form_key_split[0])
                channel = str(form_key_split[1])
                channel_num = int(channel.split('_')[-1])
                if form_key_end == 'get':
                    # Get current values from controller
                    enable, imax, iset = led_control.get_led_settings(name,channel_num)
                    imax, iset = str(imax), str(iset) 
                    have_enable = True
                    have_current = True
                    break
                if form_key_end == 'set':
                    iset = str(flask.request.form['{0}_{1}_ma'.format(name,channel)])
                    if iset:
                        have_current = True
                        if int(iset) > 1000:
                            iset = 1000
                        led_control.set_led_current(name,channel_num,int(iset))
                    if '{0}_{1}_enabled'.format(name,channel) in [str(x) for x in flask.request.form]:
                        enable = True
                    else:
                        enable = False
                    have_enable = True
                    led_control.led_enable(name,channel_num,enable)
                    break
    if have_enable:
        for n, c_vals in lighting_values:
            for c, val_dict in c_vals:
                if c == channel and n == name:
                    val_dict['enable'] = enable

    if have_current:
        for n, c_vals in lighting_values:
            for c, val_dict in c_vals:
                if c == channel and n == name:
                    val_dict['iset'] = iset 

    page_render_dict = {
            'lighting_values' : lighting_values,
            }
    render_dict.update(page_render_dict)
    return flask.render_template('tracking_2d_lighting.html', **render_dict)

# Sijax handlers
# ---------------------------------------------------------------------------------
def mode_change_handler(obj_response, new_mode):

    old_mode = redis_tools.get_str(db,'current_mode')
    new_mode = str(new_mode)

    if old_mode == new_mode:
        return

    redis_tools.set_str(db,'current_mode', new_mode)
    frame_rate = redis_tools.get_dict(db,'frame_rate_dict')['tracking_2d']
    regions_dict = redis_tools.get_dict(db,'regions_dict')

    # Stop camera triggers
    camera_trigger.stop()
    time.sleep(0.5) # wait for all frames to pass throught the system
    reset_rand_sync()

    if old_mode == 'recording' or new_mode == 'recording': 

        # Find logging and avi recording commands
        service_list = mct_introspection.get_services()
        logging_nodes = get_logging_nodes(service_list)
        recording_nodes = get_recording_nodes(service_list)
        
        if (old_mode == 'recording') and (new_mode != 'recording'):

            # Stop logging and recording
            for node in logging_nodes:
                tracking_pts_logger.stop_logging(node)
            for node in recording_nodes:
                avi_writer.stop_recording(node,'dummy.avi',frame_rate)

        if new_mode == 'recording':

            # Create sub-directory for log files.
            log_dir_base = redis_tools.get_dict(db,'logging_params_dict')['directory']
            log_dir = os.path.join(log_dir_base, datetime.datetime.now().isoformat()) 
            os.mkdir(log_dir)

            # Start loggers
            for node in logging_nodes:
                filename = '_'.join(node.split('/')[1:3])
                filename = os.path.join(log_dir, '{0}.json'.format(filename))
                tracking_pts_logger.start_logging(node,filename)

            # Start avi recordings
            for node in recording_nodes:
                filename = '_'.join(node.split('/')[1:3])
                filename = os.path.join(log_dir, '{0}.avi'.format(filename))
                avi_writer.start_recording(node,filename,frame_rate)

    if new_mode in ('preview', 'recording'):
        regions_dict = redis_tools.get_dict(db,'regions_dict')
        # Reset frame drop correctors and restart camera triggers
        frame_drop_corrector.reset_all()
        frame_drop_watchdog.reset()

        # Reset image_stitcher and three_point_tracker synchronizer for all tracking regions
        for region in regions_dict:
            image_stitcher.reset(region)
            three_point_tracker_synchronizer.reset(region)

        camera_trigger.start(frame_rate)

    # Development ----------------------------------------------------------------------------------------------
    #obj_response.html('#develop_mode_change', 'develop mode change: {0} -> {1}, {2}'.format(old_mode, new_mode))
    # ----------------------------------------------------------------------------------------------------------


def timer_update_handler(obj_response):
    """
    Callback for the timer update function. Resets the modified times for the camera 
    calibration files.
    """
    current_mode = redis_tools.get_str(db, 'current_mode')
    for mode in OPERATING_MODES:
        if mode == current_mode:
            obj_response.attr('#{0}'.format(mode),'checked','checked')
        else:
            obj_response.attr('#{0}'.format(mode),'checked','')

    # Development ------------------------------------------------------------------------------------
    #obj_response.html('#develop_timer_update', 'develop timer update: {0}'.format(str(current_mode)))
    # ------------------------------------------------------------------------------------------------

# ---------------------------------------------------------------------------------
def get_camera_assignment():
    """
    Returns the current camera assignment.
    """
    return file_tools.read_camera_assignment()

def get_transform_2d_calibration_info():
    """
    Gets the last modified date for any existing transforms 2d calibration files.
    """
    calibration_info = mct_introspection.get_transform_2d_calibration_info()
    return calibration_info

def get_camera_pairs_dict():
    camera_pairs = file_tools.read_tracking_2d_camera_pairs()
    camera_pairs_dict = {}
    for region, pairs_list in camera_pairs.iteritems():
        pair_name_list = []
        for pair in pairs_list:
            pair_name = '{0}_{1}'.format(pair[0],pair[1])
            pair_name_list.append(pair_name)
        if pair_name_list:
            camera_pairs_dict[region] = pair_name_list
    return camera_pairs_dict


def get_homography_calibration_info():
    """
    Gets the last modified date for any existing homography calibration files.
    """
    regions_dict = redis_tools.get_dict(db,'regions_dict')
    calibration_info = mct_introspection.get_homography_calibration_info()
    for region, camera_list in regions_dict.iteritems():
        for camera in camera_list:
            if not camera in calibration_info:
                calibration_info[camera] = {'modified': ''}
    return calibration_info

def get_camera_calibration_info():
    """
    Gets the last modified date for any existing homography calibration files.
    """
    regions_dict = redis_tools.get_dict(db,'regions_dict')
    calibration_info = mct_introspection.get_camera_calibration_info()
    for region, camera_list in regions_dict.iteritems():
        for camera in camera_list:
            if not camera in calibration_info:
                calibration_info[camera] = {'modified': ''}
    return calibration_info

def get_logging_nodes(service_list): 
    """
    Returns a list of all nodes which offer a logging_cmd service.
    """
    logging_srv_list = [srv for srv in service_list if 'logging_cmd' in srv.split('/')]
    logging_node_list = ['/'.join(srv.split('/')[:3]) for srv in logging_srv_list]
    return logging_node_list

def get_recording_nodes(service_list): 
    """
    Returns a list of all nodes which off a recording_cmd service.
    """
    recording_srv_list = [srv for srv in service_list if 'recording_cmd' in srv.split('/')]
    recording_node_list = ['/'.join(srv.split('/')[:4]) for srv in recording_srv_list] 
    return recording_node_list

def get_base_render_dict(): 
    """
    Creates the base render dictionary passed to all page templates.
    """
    ip_iface_ext = redis_tools.get_str(db,'ip_iface_ext')
    tab_list = get_tab_list()
    render_dict = {
            'ip_iface_ext': ip_iface_ext,
            'tab_list': tab_list,
            }
    return render_dict

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
    tab_list.append(('lighting', flask.url_for('lighting')))
    tab_list.append(('ros', flask.url_for('show_ros_info')))
    return tab_list


def get_region_mjpeg_info(region):
    """
    Returns a dictionary containing the mjpeg stream information for all
    of the images for the specified tracking region.
    """
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
    region_mjpeg_info = {}
    for v in mjpeg_info_dict.values():
        image_topic_split = v['image_topic'].split('/')
        if region == image_topic_split[1]:
            image_name = image_topic_split[2]
            region_mjpeg_info[image_name] = {
                    'image_topic' :  v['image_topic'], 
                    'mjpeg_port'  :  v['mjpeg_port'],
                    }
    return region_mjpeg_info 
        


def get_extra_video_mjpeg_info(video):
    """
    Returns a dictionary containing the mjpeg stream information for the
    extra video named video
    """
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
    extra_video_dict = redis_tools.get_dict(db,'extra_video_dict')
    extra_video_topic = extra_video_dict[video]
    extra_video_mjpeg_info = {}
    for v in mjpeg_info_dict.values():
        if extra_video_topic == v['image_topic']:
            extra_video_mjpeg_info['image_topic'] = v['image_topic']
            extra_video_mjpeg_info['mjpeg_port'] = v['mjpeg_port']
            break
    return extra_video_mjpeg_info


def get_watchdog_mjpeg_info():
    """
    Returns a dictionary containing the mjpeg stream information for the
    watchdog information image stream.
    """
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
    watchdog_mjpeg_info = {}
    for v in mjpeg_info_dict.values():
        if v['image_topic'] == '/image_frame_drop_watchdog':
            watchdog_mjpeg_info['image_topic'] = v['image_topic']
            watchdog_mjpeg_info['mjpeg_port'] = v['mjpeg_port']
            break
    return watchdog_mjpeg_info

def get_empty_lighting_values():
    """
    Gets the lighting values from the controller
    """
    lighting_values = []
    lighting_params_dict = redis_tools.get_dict(db,'lighting_params_dict')
    lighting_names_sorted = sorted(lighting_params_dict.keys())
    for name in lighting_names_sorted: 
        channel_values = []
        for channel in lighting_params_dict[name]:
            channel_num = int(channel.split('_')[1])
            values_dict = {'enable': False, 'imax': '', 'iset': ''}
            channel_values.append((channel,values_dict))
        lighting_values.append((name,channel_values))
    return lighting_values


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

    # Set operating mode
    redis_tools.set_str(db,'current_mode', 'preview')

    ## Get frame rates
    frame_rate_dict = file_tools.read_frame_rates()
    redis_tools.set_dict(db,'frame_rate_dict', frame_rate_dict)

    frame_rate = frame_rate_dict['tracking_2d']

    # Get logging directory  - add base path to directory, and create if it doesn't exist
    logging_params_dict = file_tools.read_logging_params()
    logging_params_dict['directory'] = os.path.join(
            os.environ['HOME'], 
            logging_params_dict['directory']
            )
    if not os.path.exists(logging_params_dict['directory']):
        os.mkdir(logging_params_dict['directory'])
    redis_tools.set_dict(db,'logging_params_dict', logging_params_dict)

    # Get lighting params
    mightex_params_dict = file_tools.read_mightex_params()
    lighting_params_dict = mightex_to_lighting_params(mightex_params_dict)

    redis_tools.set_dict(db,'lighting_params_dict', lighting_params_dict)

    return db

def mightex_to_lighting_params(mightex_params):
    lighting_params_dict = {}
    for name, channel_dict in mightex_params.iteritems():
        lighting_params_dict[name] = [x for x in channel_dict if x.split('_')[0] == 'channel']
        lighting_params_dict[name].sort()
    return lighting_params_dict

def cleanup():
    db.flushdb()

# ---------------------------------------------------------------------------------
if __name__ == '__main__':

    if not DEVELOP: 
        tracking_2d_node_startup()
    db = setup_redis_db()
    atexit.register(cleanup)

    if DEBUG:
        app.debug = True 
        app.run()
    else:
        app.run(host='0.0.0.0',port=5000)
