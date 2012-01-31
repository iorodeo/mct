#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_assignment')
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
from mct_camera_tools import camera_inspector_master
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

    if flask.g.sijax.is_sijax_request:
        flask.g.sijax.register_callback('assignment_change', assignment_change_handler)
        flask.g.sijax.register_callback('clear_form', clear_form_handler)
        flask.g.sijax.register_callback('assignment_save', assignment_save_handler)
        flask.g.sijax.register_callback('assignment_load', assignment_load_handler)
        flask.g.sijax.register_callback('timer_update', timer_update_handler)
        flask.g.sijax.register_callback('test', test_handler)
        return flask.g.sijax.process_request()

    else:

        scale_options = [1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2] 
        scale = flask.request.args.get('scale','0.5')
        try:
            scale = float(scale)
        except ValueError:
            scale = 0.5 

        if not scale in scale_options:
            scale = scale_options[0]

        # -------------------------------------------------------------------------
        # Computer image width - Note, you will probably want to get this from the 
        # Camera images directly.

        image_width = int(640*scale)
        image_height = int(480*scale)

        # -------------------------------------------------------------------------

        # Convert scale options and scale to strings 
        scale_options = ['{0:1.2f}'.format(x) for x in scale_options]
        scale = '{0:1.2f}'.format(scale)

        camera_assignment = redis_tools.get_dict(db,'camera_assignment')
        mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
        ip_iface_ext = redis_tools.get_str(db,'ip_iface_ext')
        select_values = create_select_values(mjpeg_info_dict)

        render_dict = {
                'mjpeg_info_dict'   : mjpeg_info_dict,
                'camera_assignment' : camera_assignment,
                'select_values'     : select_values,
                'ip_iface_ext'      : ip_iface_ext,
                'scale_options'     : scale_options,
                'scale'             : scale,
                'image_width'       : image_width,
                'image_height'      : image_height,
                }

        redis_tools.set_dict(db,'camera_assignment',camera_assignment)
        return flask.render_template('camera_view_table.html', **render_dict)


# Sijax request handlers
# -----------------------------------------------------------------------------

def assignment_change_handler(obj_response, form_values):
    """
    Handles changes to the camera assignment
    """
    camera_assignment_old = redis_tools.get_dict(db,'camera_assignment')
    camera_assignment_new = json_tools.decode_dict(form_values)
    redis_tools.set_dict(db,'camera_assignment',camera_assignment_new)
    
    message_str = ''
    for k,v in camera_assignment_new.iteritems():
        if v != camera_assignment_old[k]:
            message_str = 'Assigned camera {0} to GUID {1}'.format(v,k)
        
    obj_response.attr('#message', 'style','color:black')
    obj_response.html('#message', message_str)
    obj_response.attr('#message_table', 'style', 'display:none')

def clear_form_handler(obj_response, form_values):
    """
    Handles requests to clear form
    """
    camera_assignment = json_tools.decode_dict(form_values)
    camera_assignment = {k : '--' for k in camera_assignment}
    redis_tools.set_dict(db,'camera_assignment', camera_assignment)
    
    # Update form
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
    select_values = create_select_values(mjpeg_info_dict)
    set_camera_assignment(obj_response,camera_assignment,select_values)

    # Update message
    obj_response.attr('#message', 'style','color:black')
    obj_response.html('#message', 'Camera assignment cleared')
    obj_response.attr('#message_table', 'style', 'display:none')

def assignment_save_handler(obj_response, form_values):
    """
    Handles requests to save the current camera assignment. The assignment is 
    checked for unassigned guids and for duplicate assignments.
    """
    camera_assignment = json_tools.decode_dict(form_values)

    # Check for unassigned GUIDs
    unassigned = []
    for k,v in camera_assignment.iteritems():
        if v == '--':
            unassigned.append(k)

    # Check for duplicate values
    assignment_cnt = {}
    for k,v in camera_assignment.iteritems(): 
        try:
            assignment_cnt[v]+= 1
        except KeyError:
            assignment_cnt[v] = 1

    duplicates = {}
    for k,v in camera_assignment.iteritems():
        try:
            cnt = assignment_cnt[v]
        except KeyError:
            continue

        if cnt > 1 and v != '--':
            try:
                duplicates[v].append(k)
            except KeyError:
                duplicates[v] = [k]

    # Send response
    if unassigned:
        # Unable to save - there are unassigned cameras
        table_data = []
        for k in unassigned:
            table_data.append('<tr>')
            table_data.append('<td> <b> GUID {0} </b> </td>'.format(k))
            table_data.append('</tr>')
        table_data = '\n'.join(table_data)
        message = 'Unable to save: unassigned GUIDs'
        obj_response.attr('#message', 'style','color:red')
        obj_response.html('#message', message)
        obj_response.html('#message_table', table_data)
        obj_response.attr('#message_table', 'style', 'display:block')

    elif duplicates:
        # Unable to save - there exist duplicate camera assignemts
        table_data = []
        for k,v in duplicates.iteritems():
            table_data.append('<tr>')
            table_data.append('<td> <b> camera {0} </b> </td>'.format(k))
            table_data.append('<td> &rarr; </td>')
            table_data.append('<td> <b> {0} </b> </td>'.format(str(v)))
            table_data.append('</tr>')
        table_data = '\n'.join(table_data)
        message = 'Unable to save: duplicate assignments exist'
        obj_response.attr('#message', 'style','color:red')
        obj_response.html('#message', message)
        obj_response.html('#message_table', table_data)
        obj_response.attr('#message_table', 'style', 'display:block')

    else:
        # Everthing is OK save camera assignment
        table_data = [] 
        for k, v in camera_assignment.iteritems():
            table_data.append('<tr>')
            table_data.append('<td> <b> camera {0} </b> </td>'.format(v))
            table_data.append('<td> &rarr; </td>') 
            table_data.append('<td> <b> GUID {0} </b> </td>'.format(k))
            table_data.append('</tr>')
        table_data = '\n'.join(table_data)

        write_camera_assignment(camera_assignment)
        message = 'Camera assignment saved'
        obj_response.attr('#message', 'style','color:green')
        obj_response.html('#message', message)
        obj_response.html('#message_table', table_data)
        obj_response.attr('#message_table', 'style', 'display:block')

def assignment_load_handler(obj_response, form_values):
    """
    Load camera assignment from file.
    """
    camera_assignment_old = redis_tools.get_dict(db,'camera_assignment')
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
    select_values = create_select_values(mjpeg_info_dict)

    camera_assignment_new = read_camera_assignment()
    camera_assignment = {}
    for camera_id in camera_assignment_old:
        try:
            value = camera_assignment_new[camera_id]
        except KeyError:
            value = '--'
        if not value in select_values:
            value = '--'
        camera_assignment[camera_id] = value

    redis_tools.set_dict(db,'camera_assignment',camera_assignment)
    set_camera_assignment(obj_response,camera_assignment,select_values)
    obj_response.attr('#message', 'style','color:black')
    obj_response.html('#message', 'Current camera assignment loaded')
    obj_response.attr('#message_table', 'style', 'display:none')

def timer_update_handler(obj_response):
    """
    Updates the camera assignment form to the latest values in the database. This function
    is from a timer on the client and is used to keep multiple instances of the interface
    in sync.
    """

    camera_assignment = redis_tools.get_dict(db,'camera_assignment')
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
    select_values = create_select_values(mjpeg_info_dict)
    set_camera_assignment(obj_response, camera_assignment, select_values)

def test_handler(obj_response, form_values):
    """
    Assign a test camera assignment - for development 
    """
    # Create a camera assignment
    camera_assignment = json_tools.decode_dict(form_values)
    cnt = 0
    test_camera_assignment = {}
    for k in camera_assignment:
        cnt += 1
        test_camera_assignment[k] = str(cnt)

    # Set camera assignment values
    mjpeg_info_dict = redis_tools.get_dict(db,'mjpeg_info_dict')
    select_values = create_select_values(mjpeg_info_dict)
    set_camera_assignment(obj_response, test_camera_assignment, select_values)

    redis_tools.set_dict(db,'camera_assignment', test_camera_assignment)
    obj_response.attr('#message', 'style','color:black')
    obj_response.html('#message', 'Created test assignment')
    obj_response.attr('#message_table', 'style', 'display:none')


def set_camera_assignment(obj_response,camera_assignment,select_values):
    """
    Set the camera assignment values in the select form.
    """
    for camera_id, cur_value in camera_assignment.iteritems():
        for value in select_values:
            option_id = '#option_{0}_{1}'.format(camera_id,value)
            if cur_value == value:
                obj_response.attr(option_id,'selected','selected')
            else:
                obj_response.attr(option_id,'selected','')

# Utility functions
# ----------------------------------------------------------------------------------

def create_select_values(mjpeg_info_dict):
    """
    Creates the list of select options for camera drop down menus.
    """
    camera_numbers = [str(x) for x in range(1,len(mjpeg_info_dict)+1)]
    select_values = ['--']
    select_values.extend(camera_numbers)
    return select_values

def cleanup():
    """
    Clean up temporary redis database
    """
    db.flushdb()

def create_empty_assignment(mjpeg_info_dict):
    """
    Creates an empty camera assignment dictionary
    """
    camera_assignment = {}
    for k in mjpeg_info_dict:
        camera_assignment[k] = '--'
    return camera_assignment

def setup_redis_db():
    """
    Sets up the redis database for the camera assignemnt application
    """
    # Create db and add empty camera assignment
    db = redis.Redis('localhost',db=1)

    machine_def = mct_introspection.get_machine_def()
    ip_iface_ext = iface_tools.get_ip_addr(machine_def['mct_master']['iface-ext'])
    redis_tools.set_str(db,'ip_iface_ext',ip_iface_ext)

    mjpeg_info_dict = mjpeg_servers.get_mjpeg_info_dict()
    redis_tools.set_dict(db,'mjpeg_info_dict', mjpeg_info_dict)

    camera_assignment = create_empty_assignment(mjpeg_info_dict)
    redis_tools.set_dict(db,'camera_assignment',camera_assignment)
    return db

def write_camera_assignment(camera_assignment):
    """
    Write camera assignment to mct configuration directory
    """
    # Create dictionary for yaml file
    yaml_dict = {}
    for camera_id, value in camera_assignment.iteritems():
        camera_name = 'camera_{0}'.format(value)
        computer = mjpeg_info_dict[camera_id]['camera_computer']
        yaml_dict[camera_name] = {'guid': camera_id, 'computer': computer}

    # Write yaml file to 'camera_assignment.yaml' in cameras section of mct configuration
    config_dir = os.environ['MCT_CONFIG']
    filename = os.path.join(config_dir, 'cameras', 'camera_assignment.yaml')
    with open(filename,'w') as f:
        f.write('\n# Autogenerated configuration file - do not hand edit\n\n')
        camera_name_list = yaml_dict.keys()
        camera_name_list.sort
        yaml.dump(yaml_dict,f,default_flow_style=False)

def read_camera_assignment():
    """
    Reads the current camera assignment from the camera assignment yaml file. If 
    this file doesn't exist then None is returned.
    """
    config_dir = os.environ['MCT_CONFIG']
    filename = os.path.join(config_dir, 'cameras', 'camera_assignment.yaml')
    if os.path.isfile(filename):
        try:
            with open(filename) as f:
                data = yaml.load(f)
        except:
            return None

        camera_assignment = {}
        for k, v in data.iteritems():
            if 'camera_' in k:
                guid = v['guid']
                value = k.replace('camera_','')
                camera_assignment[guid] = value
        return camera_assignment 
    else:
        return None

def start_cameras_and_mjpeg_servers():
    """
    Starts the cameras and mjpeg servers
    """
    if not DEVELOP: 
        # Wait until camera inspectors are running and then start cameras
        while not mct_introspection.camera_inspectors_ready():
            time.sleep(0.2)
        camera_inspector_master.start_cameras()

        # Wait until the camera nodes are ready and then start the mjpeg servers
        while not mct_introspection.camera_nodes_ready():
            time.sleep(0.2)
        mjpeg_servers.start_servers()

def stop_cameras_and_mjpeg_servers():
    """
    Stops the cameras and mjpeg servers
    """
    if not DEVELOP:
        mjpeg_servers.stop_servers()
        camera_inspector_master.stop_cameras()


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

