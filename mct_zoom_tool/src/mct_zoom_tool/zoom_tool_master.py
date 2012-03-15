from __future__ import print_function
import roslib
roslib.load_manifest('mct_zoom_tool')
import rospy
from mct_msg_and_srv.srv import CommandString 


def zoom_tool_master_srv(cmd):
    proxy = rospy.ServiceProxy('/zoom_tool_master', CommandString)
    resp = proxy(cmd)
    return resp.flag, resp.message

def start():
    return zoom_tool_master_srv('start')


def stop():
    return zoom_tool_master_srv('stop')

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import sys
    cmd = sys.argv[1]
    if cmd == 'start':
        start()
    elif cmd == 'stop':
        stop()

