import roslib
roslib.load_manifest('mct_camera_trigger')
import rospy

from mct_msg_and_srv.srv import CameraTriggerCmd


def camera_trigger_cmd(cmd,freq):
    rospy.wait_for_service('camera_trigger_cmd')
    flag = None
    try:
        proxy = rospy.ServiceProxy('camera_trigger_cmd',CameraTriggerCmd)
        flag = proxy(cmd,freq)
    except rospy.ServiceException, e:
        print 'service call failed: %s'%(str(e),)
    return flag


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import sys
    cmd = sys.argv[1]
    try:
        freq = float(sys.argv[2])
        print freq
    except:
        freq = None

    flag = camera_trigger_cmd(cmd,freq)
    print flag

