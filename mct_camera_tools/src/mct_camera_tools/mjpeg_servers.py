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
    rospy.wait_for_service('mjpeg_servers')
    proxy = rospy.ServiceProxy('mjpeg_servers',CommandString)
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
    rospy.wait_for_service('mjpeg_servers_info')
    proxy = rospy.ServiceProxy('mjpeg_servers_info',GetJSONString)
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


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    if 0:
        start_servers()

    if 0:
        stop_servers()

    if 1:
        mjpeg_info_dict = get_mjpeg_info_dict()
        if mjpeg_info_dict is not None:
            for k,v in mjpeg_info_dict.iteritems():
                print(k,v)
       
