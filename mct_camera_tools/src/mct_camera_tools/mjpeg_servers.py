from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
import json
from mct_utilities import json_tools

# Services
from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import GetJSONString

def mjpeg_servers_srv(cmd):
    """
    Proxy command for the mjpeg servers start/stop command
    """
    cmd = cmd.lower()
    if not cmd in ('start', 'stop'):
        raise ValueError, "command must be 'start' or 'stop'"
    proxy = rospy.ServiceProxy('/camera_mjpeg_manager/mjpeg_servers',CommandString)
    try:
        response = proxy(cmd)
        flag = response.flag
        message = response.message
    except rospy.ServiceException, e:
        flag = False
        message = 'service request failed'
    return flag, message

def start_servers():
    """
    Starts the mjpeg servers.
    """
    return mjpeg_servers_srv('start')

def stop_servers():
    """
    Stops the mjpeg servers.
    """
    return mjpeg_servers_srv('stop')

def mjpeg_servers_info_srv():
    """
    Proxy for the mjpeg servers info service. Returns the mjpeg information
    dictionary mjpeg_info_dict or None if the mjpeg servers are not running. 
    """
    proxy = rospy.ServiceProxy('/camera_mjpeg_manager/info',GetJSONString)
    try:
        response = proxy()
        mjpeg_info_dict = json.loads(response.json_string, object_hook=json_tools.decode_dict)
    except rospy.ServiceException, e:
        mjpeg_info_dict = None
    return mjpeg_info_dict 

def get_mjpeg_info_dict():
    """
    Gets the mjpeg server dictionary from the mjpeg_manager node.
    """
    return mjpeg_servers_info_srv()

def set_transport(transport):
    """
    Set the image transport for the mjpeg servers and the throttling nodes.
    """
    proxy = rospy.ServiceProxy('/camera_mjpeg_manager/set_transport', CommandString)
    resp = proxy(transport)
    return resp.flag, resp.message

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    if 0:
        start_servers()

    if 0:
        stop_servers()

    if 1:
        set_transport('image_rect')

    if 0:
        mjpeg_info_dict = get_mjpeg_info_dict()
        if mjpeg_info_dict is not None:
            for k,v in mjpeg_info_dict.iteritems():
                print(k,v)
       
