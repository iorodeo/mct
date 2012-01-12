from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy

from mct_msg_and_srv.srv import MasterInspectorCameras

def inspector_camera_srv(cmd):
    rospy.wait_for_service('master_inspector_cameras')
    proxy = rospy.ServiceProxy('master_inspector_cameras',MasterInspectorCameras)
    response = False
    try:
        response = proxy(cmd)
    except rospy.ServiceException, e:
        print('ERROR: service request failed, {0}'.format(str(e)))
    return response


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import sys

    cmd = sys.argv[1]
    cmd = cmd.lower()
    
    if not cmd in ('start', 'stop'):
        print("ERROR: command must be 'start' or 'stop'")

    inspector_camera_srv(cmd)

    


