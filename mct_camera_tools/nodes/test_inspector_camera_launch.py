from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
from mct_msg_and_srv.srv import CommandString 

def inspector_camera_srv(cmd):
    rospy.wait_for_service('master_inspector_cameras')
    proxy = rospy.ServiceProxy('master_inspector_cameras',CommandString)
    response = None 
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
    response = inspector_camera_srv(cmd)
    print(response)

    


