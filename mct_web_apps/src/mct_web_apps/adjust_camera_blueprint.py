#!/usr/bin/env python
from __future__ import print_function
import flask

adjust_camera = flask.Blueprint('adjust_camera', __name__, template_folder='templates')

@adjust_camera.route('/adjust/<camera>', methods=['GET', 'POST'])
def adjust_camera_page(camera):
    return 'adjust camera = {0}'.format(camera)
