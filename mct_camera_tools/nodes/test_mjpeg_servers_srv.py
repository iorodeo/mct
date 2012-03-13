from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
from mct_msg_and_srv.srv import CommandString 
from mct_camera_tools import mjpeg_servers


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import sys
    cmd = sys.argv[1]
    cmd = cmd.lower()
    response = mjpeg_servers.mjpeg_servers_srv(cmd)
    print(response)

