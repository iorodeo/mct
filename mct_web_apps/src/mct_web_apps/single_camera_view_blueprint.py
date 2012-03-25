#!/usr/bin/env python
from __future__ import print_function
import flask
import redis
import config
import common_args
import common_tasks

from mct_utilities import redis_tools
from mct_utilities import iface_tools


# Create db and add empty camera assignment
db = redis.Redis('localhost',db=config.redis_db)

single_camera_view = flask.Blueprint('single_camera_view', __name__, template_folder='templates')

@single_camera_view.route('/<camera>')
def page(camera):
    camera = str(camera)
    scale, scale_options = common_args.get_scale(config,flask.request)
    redis_tools.set_str(db,'scale', scale)
    image_width, image_height = common_tasks.get_image_size(scale)
    ip_iface_ext = redis_tools.get_str(db,'ip_iface_ext')

    try:
        mjpeg_info = mjpeg_info_dict[camera]
    except KeyError:
        return

    render_dict = {
            'scale'             : scale,
            'scale_options'     : scale_options,
            'image_width'       : image_width,
            'image_height'      : image_height,
            'ip_iface_ext'      : ip_iface_ext,
            'camera'            : camera,
            'mjpeg_info'        : mjpeg_info,
            }

    return flask.render_template('single_camera_view.html',**render_dict)
