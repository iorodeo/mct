from __future__ import print_function
import roslib
roslib.load_manifest('mct_introspection')
import rospy
import os
import os.path
import yaml
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

    Note, assumes that the camera inspector nodes are running.
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

def get_number_of_camera_inspectors():
    """
    Returns the number of currently running camera inspector nodes.
    """
    node_list = get_nodes()
    node_list = [node for node in node_list if 'inspector' in node]
    node_list = [node for node in node_list if 'inspector_master' not in node]
    return len(node_list)

def camera_inspectors_ready():
    """
    Check to see if the camera inspector nodes are ready. The inspector nodes
    are considered to be ready when the number of camera inspector nodes is
    equal to the number of machine in the system. 
    """
    number_of_inspectors = get_number_of_camera_inspectors()
    number_of_machines = get_number_of_machines()
    if number_of_inspectors == number_of_machines:
        return True
    else:
        return False

def get_number_of_machines():
    """
    Returns the number of machines in the system.
    """
    machine_def = get_machine_def()
    machine_def.pop('user')
    return len(machine_def)

def get_number_of_cameras():
    """
    Returns the number of cameras attached to the system.

    Note, assumes that the camera inspector nodes are running.
    """
    if camera_inspectors_ready():
        camera_dict = find_cameras()
        return len(camera_dict)
    else:
        return None

def get_number_of_camera_nodes():
    """
    Returns the number of camera nodes currently running. Assumes that the
    camera nodes have the names camera and node in them.
    """
    node_list = get_nodes()
    node_list = [node for node in node_list if 'camera' and 'node' in node]
    return len(node_list)

def get_number_of_camera_topics(transport='image_raw'):
    """
    Returns the number of camera topics
    """
    camera_topics = find_camera_topics(transport=transport)
    return len(camera_topics)

def camera_nodes_ready():
    """
    Checks to see if the camera nodes are ready. The camera nodes are
    considered ready when the number of camera topics (with transport
    raw_image) is equal to the number of cameras attached to the system.
    """
    number_of_cameras = get_number_of_cameras()
    number_of_camera_topics = get_number_of_camera_topics()
    if number_of_cameras == number_of_camera_topics:
        return True
    else:
        return False

def get_machine_def():
    """
    Reads the machine definition file.
    """
    config_pkg = os.environ['MCT_CONFIG']
    machine_def_file = os.path.join(config_pkg,'machine','machine_def.yaml')
    with open(machine_def_file,'r') as f:
        machine_def = yaml.load(f)
    return machine_def

def get_slave_info():
    """
    Reads the machine definition file and gets the slave information.
    """
    slave_info = get_machine_def()
    slave_info.pop('mct_master') 
    slave_info.pop('user')
    return slave_info

def get_hosts():
    """
    Returns a list of all hosts in the current machine
    """
    machine_def = get_machine_def()
    hosts = []
    for k,v in machine_def.iteritems():
        try:
            hosts.append(v['address'])
        except KeyError:
            pass
        except TypeError:
            pass
    return hosts

def get_slave_hosts():
    """
    Returns a list of the slave host names
    """
    slave_info = get_slave_info()
    return [v['address'] for k,v in slave_info.iteritems()]

def get_slave_macs():
    """
    Return a list of the slave mac addresses
    """
    slave_info = get_slave_info()
    return [v['mac'] for k,v in slave_info.iteritems()]

def get_slave_mac_and_iface():
    """
    Returns a dictionary with the slave mac addresses and 
    """
    slave_info = get_slave_info()
    return [(v['mac'], v['iface']) for k,v in slave_info.iteritems()]

def get_master():
    """
    Get the host name of the master computer.
    """
    machine_def = get_machine_def()
    return machine_def['mct_master']['address']

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    if 0:
        node_list = get_nodes()
        print('node_list:')
        print(node_list)
        print()

    if 0:
        service_list = get_services()
        print('service_list:')
        print(service_list)
        print()

    if 0:
        camera_dict = find_cameras()
        print('camera_dict:')
        for k,v in camera_dict.iteritems():
            print(k,v)
            print()

    if 0:
        camera_dict = find_cameras(add_info=True)
        print('camera_dict:')
        for k,v in camera_dict.iteritems():
            print(k,v)
            print()

    if 0:
        camera_topics = find_camera_topics()
        print('camera_topics:')
        for topic in camera_topics:
            print(topic)

    if 0:
        num = get_number_of_camera_inspectors()
        print('number of camera inspectors: {0}'.format(num))

    if 0:
        num = get_number_of_machines()
        print('number of machines: {0}'.format(num))

    if 0:
        ready = camera_inspectors_ready()
        print('camera inspectors ready: {0}'.format(ready))

    if 0:
        num = get_number_of_cameras()
        print('number of cameras: {0}'.format(num))

    if 0:
        num = get_number_of_camera_nodes()
        print('number of camera nodes: {0}'.format(num))

    if 1:
        num = get_number_of_camera_topics()
        print('number of camera topics: {0}'.format(num))

    if 0:
        ready = camera_nodes_ready()
        print('camera nodes ready: {0}'.format(ready))

    if 0: 
        machine_def = get_machine_def()
        print('machine_def:\n {0}'.format(machine_def))

    if 0: 
        slave_info = get_slave_info()
        print('slave_info:\n {0}'.format(slave_info))

    if 0: 
        slave_hosts = get_slave_hosts()
        print('slave_hosts:\n {0}'.format(slave_hosts))

    if 0: 
        slave_macs = get_slave_macs()
        print('slave_macs:\n {0}'.format(slave_macs))

    if 0: 
        slave_mac_and_iface = get_slave_mac_and_iface()
        print('slave_mac_and_iface:\n {0}'.format(slave_mac_and_iface))

    if 0: 
        hosts = get_hosts()
        print('hosts:\n {0}'.format(hosts))

            
