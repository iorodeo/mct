from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
import json
from mct_introspection import get_services

# Services
from mct_msg_and_srv.srv import FindCamera1394

def find_cameras():
    service_list = get_services()
    camera_dict = {}
    for service in service_list:
        service_name = service.split('/')[-1]
        if service_name != 'find_camera1394':
            continue
        rospy.wait_for_service(service)
        find_cameras_proxy = rospy.ServiceProxy(service,FindCamera1394)
        camera_dict_temp = None
        try:
            response = find_cameras_proxy()
            camera_dict_temp = json.loads(response.camera_info_json)
        except rospy.ServiceException, e:
            print('service request failed {0}'.format(str(e)))
        if camera_dict_temp is not None:
            camera_dict.update(camera_dict_temp)
    return camera_dict

def find_cameras_info():
    service_list = get_services()
    camera_dict = {}
    for service in service_list:
        service_name = service.split('/')[-1]
        if service_name != 'find_camera1394_info':
            continue
        rospy.wait_for_service(service)
        find_cameras_proxy = rospy.ServiceProxy(service,FindCamera1394)
        camera_dict_temp = None
        try:
            response = find_cameras_proxy()
            camera_dict_temp = json.loads(response.camera_info_json)
        except rospy.ServiceException, e:
            print('service request failed {0}'.format(str(e)))
        if camera_dict_temp is not None:
            camera_dict.update(camera_dict_temp)
    return camera_dict


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    from mct_camera_tools.camera1394_inspector import printGUIDDict

    if 1:
        camera_dict = find_cameras()
        print()
        printGUIDDict(camera_dict)
        print()


    if 0:
        print('-'*80)
        camera_dict = find_cameras_info()
        print() 
        printGUIDDict(camera_dict)
        print() 

