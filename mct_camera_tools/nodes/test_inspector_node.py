import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
import json
from mct_camera_tools.camera1394_inspector import printGUIDDict

# Services
from mct_msg_and_srv.srv import FindCamera1394

def find_cameras():
    rospy.wait_for_service('find_camera_1394')
    find_cameras_proxy = rospy.ServiceProxy('find_camera_1394',FindCamera1394)
    camera_dict = None
    try:
        response = find_cameras_proxy()
        camera_dict = json.loads(response.camera_info_json)
    except rospy.ServiceException, e:
        print 'service request failed %s'%(str(e),)
    return camera_dict

def find_cameras_info():
    rospy.wait_for_service('find_camera_1394_info')
    find_cameras_proxy = rospy.ServiceProxy('find_camera_1394_info',FindCamera1394)
    camera_dict = None
    try:
        response = find_cameras_proxy()
        camera_dict = json.loads(response.camera_info_json)
    except rospy.ServiceException, e:
        print 'service request failed %s'%(str(e),)
    return camera_dict


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    camera_dict = find_cameras()
    print 
    printGUIDDict(camera_dict)
    print 
    print '-'*80
    
    camera_dict = find_cameras_info()
    print 
    printGUIDDict(camera_dict)
    print 

