#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_assignment')
import rospy
import os
import flask
import flask_sijax
import redis
import atexit

from mct_camera_tools import mjpeg_servers
from mct_utilities import redis_tools
from mct_utilities import json_tools

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
        return flask.g.sijax.process_request()

    else:
        camera_assignment = redis_tools.get_dict(db,'camera_assignment')
        mjpeg_info_dict = mjpeg_servers.get_mjpeg_info_dict()
        select_values = create_select_values(mjpeg_info_dict)
        render_dict = {
                'mjpeg_info_dict'   : mjpeg_info_dict,
                'camera_assignment' : camera_assignment,
                'select_values'     : select_values,
                }

        redis_tools.set_dict(db,'camera_assignment',camera_assignment)
        return flask.render_template('camera_view_table.html', **render_dict)


def assignment_change_handler(obj_response, formValues):
    """
    Handles changes to the camera assignment
    """
    camera_assignment_old = redis_tools.get_dict(db,'camera_assignment')
    camera_assignment_new = json_tools.decode_dict(formValues)
    redis_tools.set_dict(db,'camera_assignment',camera_assignment_new)
    
    message_str = ''
    for k,v in camera_assignment_new.iteritems():
        if v != camera_assignment_old[k]:
            message_str = 'GUID {0} --> Camera {1}'.format(k,v)
        
    obj_response.html('#message', message_str)

def clear_form_handler(obj_response, formValues):
    """
    Handles requests to clear form
    """
    camera_assignment = json_tools.decode_dict(formValues)
    camera_assignment = {k : '--' for k in camera_assignment}
    redis_tools.set_dict(db,'camera_assignment', camera_assignment)
    
    # Update form
    mjpeg_info_dict = mjpeg_servers.get_mjpeg_info_dict()
    select_values = create_select_values(mjpeg_info_dict)
    for camera_id in mjpeg_info_dict:
        for value in select_values:
            option_id = '#option_{0}_{1}'.format(camera_id,value)
            if value == '--':
                obj_response.attr(option_id,'selected','selected')
            else:
                obj_response.attr(option_id,'selected','')

    # Update message
    obj_response.html('#message', 'Camera assignment cleared')

def assignment_save_handler(obj_response, formValues):
    camera_assignment = json_tools.decode_dict(formValues)

    # Check for unassigned GUIDs
    unassigned = []
    for k,v in camera_assignment.iteritems():
        if v == '--':
            unassigned.append(k)

    # Check for duplicate values
    values_cnt = {}
    for k,v in camera_assignment.iteritems():
        try:
            values_cnt[v]+= 1
        except KeyError:
            values_cnt[v] = 1

    # --------------------------------------------------------------------------
    # Not done - finished checking for duplicate values
    # --------------------------------------------------------------------------

    if unassigned:
        unassigned_str = '<br>'.join(unassigned)
        message = 'Unable to save: unassigned GUIDs <br><br>{0}'.format(unassigned_str)
    else:
        message = 'Camera assignment saved'
    obj_response.html('#message', message)

# Utility functions
# ----------------------------------------------------------------------------------

def create_select_values(mjpeg_info_dict):
    camera_numbers = [str(x) for x in range(1,len(mjpeg_info_dict)+1)]
    select_values = ['--']
    select_values.extend(camera_numbers)
    return select_values


def cleanup_db():
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


# Create db and add empty camera assignment
db = redis.Redis('localhost',db=1)
mjpeg_info_dict = mjpeg_servers.get_mjpeg_info_dict()
camera_assignment = create_empty_assignment(mjpeg_info_dict)
redis_tools.set_dict(db,'camera_assignment',camera_assignment)

# ----------------------------------------------------------------------------------
if __name__ == '__main__':

    app.debug = True
    app.run()

