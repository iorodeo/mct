import roslib
roslib.load_manifest('lia_avi_writer')
import rospy
from lia_services.srv import RecordingCmd

def test_recording_cmd(cmd):
    #rospy.wait_for_service('recording_cmd')
    recording_cmd_proxy = rospy.ServiceProxy('recording_cmd',RecordingCmd)
    try:
        response = recording_cmd_proxy(cmd,'dummy_file.avi', 5.0)
        print response
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%(e,)

# ----------------------------------------------------------------------------
if __name__ == '__main__':
    import sys
    cmd = sys.argv[1]
    test_recording_cmd(cmd)
    
