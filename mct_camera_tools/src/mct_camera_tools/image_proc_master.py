from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
from mct_msg_and_srv.srv import CommandString 

def image_proc_master_srv(cmd):
    service = 'image_proc_master'
    rospy.wait_for_service(service)
    proxy = rospy.ServiceProxy(service,CommandString)
    response = proxy(cmd)
    return response

def start_image_proc():
    return image_proc_master_srv('start')

def stop_image_proc():
    return image_proc_master_srv('stop')
