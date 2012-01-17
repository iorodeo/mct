#!/usr/bin/env python
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
import json
from mct_camera_tools.camera1394_inspector import Camera1394Inspector

# Services
from mct_msg_and_srv.srv import GetJSONString 
from mct_msg_and_srv.srv import GetJSONStringResponse 

class Inspector_Node(object):

    def __init__(self):
        self.cam_inspector = Camera1394Inspector()
        self.find_cameras_srv = rospy.Service(
                'find_camera1394',
                GetJSONString,
                self.handle_find_cameras
                )
        self.get_camera_info_srv = rospy.Service(
                'find_camera1394_info',
                GetJSONString,
                self.handle_find_cameras_info
                )
        rospy.init_node('camera1394_inspector')

    def run(self):
        rospy.spin()

    def handle_find_cameras(self,req):
        camera_dict = self.cam_inspector.getGUIDDict(info=False)
        camera_data_json = json.dumps(camera_dict)
        return GetJSONStringResponse(camera_data_json)

    def handle_find_cameras_info(self,req):
        camera_dict = self.cam_inspector.getGUIDDict(info=True)
        camera_data_json = json.dumps(camera_dict)
        return GetJSONStringResponse(camera_data_json)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    node = Inspector_Node()
    node.run()


