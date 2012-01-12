from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
import json
from mct_introspection import get_services
from mct_introspection import find_cameras

def display_camera_dict(camera_dict,have_printGUIDDict=False):
    if have_printGUIDDict:
        printGUIDDict(camera_dict)
    else:
        for k,v in camera_dict.iteritems():
            print(k, v)

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    try:
        from mct_camera_tools.camera1394_inspector import printGUIDDict
        have_printGUIDDict = True
    except ImportError:
        have_printGUIDDict = False

    if 1:
        camera_dict = find_cameras()
        print()
        display_camera_dict(camera_dict,have_printGUIDDict)
        print()

    if 1:
        print('-'*80)
        print() 
        camera_dict = find_cameras(add_info=True)
        display_camera_dict(camera_dict,have_printGUIDDict)
        print() 

