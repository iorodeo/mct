from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy

import mct_introspection
from mct_utilities import file_tools

from mct_msg_and_srv.srv import SetCameraParam
from mct_msg_and_srv.srv import GetCameraParam


def set_camera_param(camera, param):
    """
    Proxy function for setting the camera parameters.
    """
    proxy = rospy.ServiceProxy('set_camera_param', SetCameraParam)
    brightness = param['brightness']
    shutter = param['shutter']
    gain = param['gain']
    try:
        rsp = proxy(camera, brightness, shutter, gain)
        flag = rsp.flag
        message = rsp.message
    except rospy.ServiceException, e:
        flag = False
        message = 'ERROR: unable to set camera parameters: {0}'.format(str(e))
        print(message)
    return flag, message

    
def get_camera_param(camera):
    """
    Proxy function for getting the camera parameters.
    """
    param = {}
    proxy = rospy.ServiceProxy('get_camera_param', GetCameraParam)
    try:
        rsp = proxy(camera)
        flag = rsp.flag
        message = rsp.message
        param['brightness'] = rsp.brightness
        param['shutter' ] = rsp.shutter
        param['gain'] = rsp.gain
    except rospy.ServiceException, e:
        flag = False
        message = 'ERROR: unable to get camera parameers: {0}'.format(str(e))
        param['brightness'] = 0.0 
        param['shutter' ] = 0.0
        param['gain'] = 0.0 
    return param, flag, message


def set_camera_param_from_file(camera=None):
    """
    Sets the given camera parametes from the parameters file in the mct
    configuration directory. If camera is none then sets the camera parameters
    for all cameras.
    """
    if not camera is None:
        camera_list = [camera]
    else:
        camera_list = get_camera_list()

    for camera in camera_list:
        param = file_tools.read_camera_params(camera)
        set_camera_param(camera, param)

def get_all_camera_param():
    """
    Returns a dictionary of all camera parameters.
    """
    camera_list = get_camera_list()
    camera_param_dict ={}
    for camera in camera_list:
        param, flag, messages = get_camera_param(camera)
        camera_param_dict[camera] = param
    return camera_param_dict
    
def get_camera_list(): 
    """
    Returns list of cameras.
    """
    camera_nodes = mct_introspection.get_camera_nodes()
    camera_list = []
    for node in camera_nodes:
        camera_list.append(node.split('/')[2])
    return camera_list

# -----------------------------------------------------------------------------
if __name__ == '__main__':


    if 0:
        camera = 'camera_1'
        param_orig, flag, message = get_camera_param(camera)
        print('param_orig', param_orig)
        print('flag', flag)
        print('message', message)
        print()

        param_new_req = {'brightness': 700, 'shutter': 200, 'gain' : 150}
        flag, message = set_camera_param(camera, param_new_req)

        param_new_rsp, flag, message = get_camera_param(camera)
        print('param_req', param_new_req)
        print('param_rsp', param_new_rsp)
        print('flag', flag)
        print('message', message)
        print()

        flag, message = set_camera_param(camera, param_orig)
        param_final, flag, message = get_camera_param(camera)
        print('param_final', param_final)
        print('flag', flag)
        print('message', message)
        print()

    if 0:
        set_camera_param_from_file()

    if 1:
        param_dict = get_all_camera_param()
        for k, v in param_dict.iteritems():
            print(k,v)



    


