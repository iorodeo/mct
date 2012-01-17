from __future__ import print_function
import roslib
roslib.load_manifest('mct_introspection')
import rospy
import json
import subprocess
from mct_utilities import json_tools

# Services
from mct_msg_and_srv.srv import GetJSONString 

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
        find_cameras_proxy = rospy.ServiceProxy(service,GetJSONString)
        camera_dict_temp = None
        try:
            response = find_cameras_proxy()
            camera_dict_temp = json.loads(response.json_string,object_hook=json_tools.decode_dict)
        except rospy.ServiceException, e:
            print('service request failed {0}'.format(str(e)))
        if camera_dict_temp is not None:
            if add_machine:
                add_machine2camera_dict(camera_dict_temp, machine)
            camera_dict.update(camera_dict_temp)
    return camera_dict

def find_camera_topics(transport='image_raw'):
    """
    Finds a list of all active camera topics
    """
    topic_list = rospy.get_published_topics()
    camera_topic_list = []
    for topic in topic_list:
        topic_name, topic_type= topic
        topic_name_split = topic_name.split('/')
        if (topic_name_split[-1] == transport) and ('camera' in topic_name_split):
            camera_topic_list.append(topic_name)
    return camera_topic_list
        

def add_machine2camera_dict(input_dict, machine):
    """
    Adds machine name to camera info dictionary.
    """
    output_dict = {}
    for k,v in input_dict.iteritems():
        v['machine'] = machine

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

    if 0:
        camera_dict = find_cameras(add_info=True)
        for k,v in camera_dict.iteritems():
            print(k,v)
            print()

    if 0:
        camera_topics = find_camera_topics()
        for topic in camera_topics:
            print(topic)
        
            
