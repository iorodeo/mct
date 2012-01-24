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

# Setup application w/ sijax
app = flask.Flask(__name__)
app.config["SIJAX_STATIC_PATH"] = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')
app.config["SIJAX_JSON_URI"] = '/static/js/sijax/json2.js'
flask_sijax.Sijax(app)

# Routes
# ----------------------------------------------------------------------------------

@flask_sijax.route(app, '/', methods=['GET','POST'])
def index():

    camera_assignment = redis_tools.get_dict(db,'camera_assignment')
    mjpeg_info_dict = mjpeg_servers.get_mjpeg_info_dict()
    camera_numbers = [str(x) for x in range(1,len(mjpeg_info_dict)+1)]
    select_values = ['--']
    select_values.extend(camera_numbers)

    

    # Another Sijax handler function which receives no arguments
    def test_handler(obj_response):
        obj_response.alert('Goodbye, whoever you are.')

    if flask.g.sijax.is_sijax_request:
        flask.g.sijax.register_callback('test', test_handler)
        return flask.g.sijax.process_request()


    # Handle Post request
    if flask.request.method == 'POST':
        if 'save_button' in flask.request.form:
            for camera_id in mjpeg_info_dict:
                camera_assignment[camera_id] = str(flask.request.form[camera_id])
        elif 'clear_button' in flask.request.form:
            camera_assignment = create_empty_assignment(mjpeg_info_dict)
            
       
    render_dict = {
            'mjpeg_info_dict'   : mjpeg_info_dict,
            'camera_assignment' : camera_assignment,
            'select_values'     : select_values,
            }

    redis_tools.set_dict(db,'camera_assignment',camera_assignment)
    return flask.render_template('camera_view_table.html', **render_dict)


# Utility functions
# ----------------------------------------------------------------------------------

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

