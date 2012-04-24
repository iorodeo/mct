from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy

from mct_msg_and_srv.srv import SetCameraParam
from mct_msg_and_srv.srv import GetCameraParam


