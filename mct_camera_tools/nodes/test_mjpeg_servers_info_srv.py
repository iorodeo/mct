from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
import json

from mct_utilities import json_tools
from mct_camera_tools import mjpeg_servers
from mct_msg_and_srv.srv import GetJSONString 

if __name__ == '__main__':

    mjpeg_info_dict = mjpeg_servers.mjpeg_servers_info_srv()
    if mjpeg_info_dict is not None:
        for k,v in mjpeg_info_dict.iteritems():
            print(k,v)
    else:
        print('None')
