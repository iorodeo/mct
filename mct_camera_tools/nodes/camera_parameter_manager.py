#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import threading
import rospy

import dynamic_reconfigure.client
import mct_introspection

from mct_msg_and_srv.srv import SetCameraParam
from mct_msg_and_srv.srv import SetCameraParamResponse
from mct_msg_and_srv.srv import GetCameraParam
from mct_msg_and_srv.srv import GetCameraParamResponse


class CameraParameterManager(object):

    """
    Gets and sets the camera parameters using the dynamics reconfigure client. 
    """

    def __init__(self):
        self.ready = False
        rospy.init_node('camera_parameter_manager')

        self.set_camera_param_srv = rospy.Service(
                'set_camera_param', 
                SetCameraParam, 
                self.handle_set_camera_param
                )
        self.get_camera_params_srv = rospy.Service(
                'get_camera_param',
                GetCameraParam,
                self.handle_get_camera_param
                )
        self.ready = True

    def handle_set_camera_param(self,req):
        """
        Handles requests to set a given cameras parameters. Currenty,
        brightness, shutter, and gain can be set.
        """
        response = SetCameraParamResponse()
        response.flag = True
        response.message = ''

        node = get_node_from_camera_name(req.camera_name)
        if node is not None:
            params = {
                    'brightness': req.brightness,
                    'shutter': req.shutter,
                    'gain': req.gain,
                    }
            client = dynamic_reconfigure.client.Client(node)
            client.update_configuration(params)
        else:
            response.flag = False
            response.message = 'camera, {0},  not found'.format(req.camera_name)

        return response

    def handle_get_camera_param(self,req):
        """
        Handles requests to get the parameters for a given camera.
        """
        response = GetCameraParamResponse()
        response.flag = True
        response.message = ''

        node = get_node_from_camera_name(req.camera_name)
        if node is not None:
            client = dynamic_reconfigure.client.Client(node)
            config = client.get_configuration()
            response.brightness = config['brightness']
            response.shutter = config['shutter']
            response.gain = config['gain']
        else:
            response.flag = False
            response.message = 'camera, {0},  not found'.format(req.camera_name)

        return response 

    def run(self):
        rospy.spin()

def get_node_from_camera_name(camera_name):
    camera_nodes = mct_introspection.get_camera_nodes()
    if camera_name in camera_nodes:
        return camera_name
    for node in camera_nodes:
        if camera_name in node.split('/'):
            return node
    return None

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    node = CameraParameterManager()
    node.run()
