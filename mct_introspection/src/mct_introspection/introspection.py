from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
import json
import subprocess

# Services
from mct_msg_and_srv.srv import FindCamera1394

def get_nodes():
    """
    Returns a list of all currently running nodes.
    """
    node_str = subprocess.check_output(['rosnode', 'list'])
    node_list = node_str.split()
    return node_list

def get_services():
    """
    Returns a list of all currently available services.
    """
    service_str = subprocess.check_output(['rosservice','list'])
    service_list = service_str.split()
    return service_list

def find_cameras(add_machine=True, add_info=False):
    """
    Find all cameras connected to the master and slave computers.

    Returns a dictionary with camera guid as keys and the sub-dictionary
    as the value. The sub-dictionary contains the model, vendor and unit 
    number.
    """
    if add_info:
        target_service_name = 'find_camera1394_info'
    else:
        target_service_name = 'find_camera1394'

    service_list = get_services()
    camera_dict = {}
    for service in service_list:
        service_parts= service.split('/')
        service_name = service_parts[-1]
        if service_name != target_service_name:
            continue
        machine = service_parts[1]
        rospy.wait_for_service(service)
        find_cameras_proxy = rospy.ServiceProxy(service,FindCamera1394)
        camera_dict_temp = None
        try:
            response = find_cameras_proxy()
            camera_dict_temp = json.loads(response.camera_info_json)
        except rospy.ServiceException, e:
            print('service request failed {0}'.format(str(e)))
        if camera_dict_temp is not None:
            camera_dict_temp = dict_unicode2str(camera_dict_temp)
            if add_machine:
                add_machine2camera_dict(camera_dict_temp, machine)
            camera_dict.update(camera_dict_temp)
    return camera_dict


def add_machine2camera_dict(input_dict, machine):
    """
    Adds machine name to camera info dictionary.
    """
    output_dict = {}
    for k,v in input_dict.iteritems():
        v['machine'] = machine

def dict_unicode2str(dict_input):
    """
    Convert all unicode strings in the dictionary to normal strings.
    """
    dict_output = {}
    for k,v in dict_input.iteritems():
        if type(k) == unicode:
            k = str(k)
        if type(v) == unicode:
            v = str(v)
        if type(v) == dict:
            v = dict_unicode2str(v)
        if type(v) == list:
            v = list_unicode2str(v)
        dict_output[k] = v
    return dict_output

def list_unicode2str(list_input):
    """
    Convert all unicode strings in a list to normal strings.
    """
    list_output = []
    for v in list_input:
        if type(v) == unicode:
            v = str(v)
        if type(v) == list:
            v = list_unicode2str(v)
        if type(v) == dict:
            v = dict_unicode2str(v)
        list_output.append(v)
    return list_output

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    if 0:
        node_list = get_nodes()
        print(node_list)
        print()

    if 0:
        service_list = get_services()
        print(service_list)
        print()

    if 1:
        camera_dict = find_cameras()
        for k,v in camera_dict.iteritems():
            print(k,v)
            print()

    if 1:
        camera_dict = find_cameras(add_info=True)
        for k,v in camera_dict.iteritems():
            print(k,v)
            print()
            
