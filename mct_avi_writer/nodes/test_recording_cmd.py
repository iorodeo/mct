import roslib
roslib.load_manifest('mct_avi_writer')
import rospy
from mct_msg_and_srv.srv import RecordingCmd

def test_recording_cmd(cmd):
    recording_cmd_proxy = rospy.ServiceProxy('recording_cmd',RecordingCmd)
    try:
        response = recording_cmd_proxy(cmd,'dummy_file.avi', 15.0)
        print response
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%(e,)

# ----------------------------------------------------------------------------
if __name__ == '__main__':
    import sys
    cmd = sys.argv[1]
    test_recording_cmd(cmd)
    
