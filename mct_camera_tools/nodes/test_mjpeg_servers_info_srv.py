from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
import json
from mct_utilities import json_tools

from mct_msg_and_srv.srv import GetJSONString 


def mjpeg_servers_info_srv():
    rospy.wait_for_service('camera_mjpeg_servers_info')
    proxy = rospy.ServiceProxy('camera_mjpeg_servers_info',GetJSONString)
    try:
        response = proxy()
        mjpeg_info_dict = json.loads(response.json_string, object_hook=json_tools.decode_dict)
    except rospy.ServiceException, e:
        mjpeg_info_dict = None
    return mjpeg_info_dict 

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    mjpeg_info_dict = mjpeg_servers_info_srv()
    if mjpeg_info_dict is not None:
        for k,v in mjpeg_info_dict.iteritems():
            print(k,v)
    else:
        print('None')
