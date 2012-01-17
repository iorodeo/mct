from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy

# Services
from mct_msg_and_srv.srv import CommandString 

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

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    if 0:
        start_servers()

    if 1:
        stop_servers()
