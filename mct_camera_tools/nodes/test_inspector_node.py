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

    for service in service_list:
        service_name = service.split('/')[-1]
        if service_name == 'find_camera1394':
            print(service)

        ##rospy.wait_for_service('/slave1/find_camera_1394')
        ##find_cameras_proxy = rospy.ServiceProxy('/slave1/find_camera_1394',FindCamera1394)
        #rospy.wait_for_service(service)
        #find_cameras_proxy = rospy.ServiceProxy(service,FindCamera1394)
        #camera_dict = None
        #try:
        #    response = find_cameras_proxy()
        #    camera_dict = json.loads(response.camera_info_json)
        #    print(camera_dict)
        #except rospy.ServiceException, e:
        #    print('service request failed {0}'.format(str(e)))
        ##return camera_dict

def find_cameras_info():
    rospy.wait_for_service('/slave1/find_camera_1394_info')
    find_cameras_proxy = rospy.ServiceProxy('/slave1/find_camera_1394_info',FindCamera1394)
    camera_dict = None
    try:
        response = find_cameras_proxy()
        camera_dict = json.loads(response.camera_info_json)
    except rospy.ServiceException, e:
        print('service request failed {0}'.format(str(e)))
    return camera_dict


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    from mct_camera_tools.camera1394_inspector import printGUIDDict

    if 1:
        camera_dict = find_cameras()
        #print()
        #printGUIDDict(camera_dict)
        #print()


    if 0:
        print('-'*80)
        camera_dict = find_cameras_info()
        print() 
        printGUIDDict(camera_dict)
        print() 

