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
from mct_utilities import file_tools

# Services
from mct_msg_and_srv.srv import GetJSONString 

def get_nodes():
    """
    Returns a list of all currently running nodes.
    """
    node_str = subprocess.check_output(['rosnode', 'list'])
    node_list = node_str.split()
    return node_list

def exists_node(node):
    """
    Returns True if node is in list of currently running nodes
    """
    node_list = get_nodes()
    if node in node_list:
        return True
    else:
        return False

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

def find_camera_image_topics(transport='image_raw'):
    """
    Finds a list of all active camera topics
    """
    topic_list = rospy.get_published_topics()
    image_topic_list = []
    for topic in topic_list:
        topic_name, topic_type= topic
        topic_name_split = topic_name.split('/')
        if (topic_name_split[-1] == transport) and ('camera' in topic_name_split):
            image_topic_list.append(topic_name)
    return image_topic_list

def find_topics_w_name(name):
    """
    Finds a list of all topics containing the given name
    """
    topic_list = [n for n,t in rospy.get_published_topics()]
    topic_w_name_list = []
    for topic in topic_list:
        if name in topic.split('/'):
            topic_w_name_list.append(topic)
    return topic_w_name_list

def find_topics_w_ending(name):
    """
    Finds a list of all topics ending in the given name
    """
    topic_list = [n for n,t in rospy.get_published_topics()]
    topic_w_name_list = []
    for topic in topic_list:
        if name  ==  topic.split('/')[-1]:
            topic_w_name_list.append(topic)
    return topic_w_name_list

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

def get_number_of_camera_image_topics(transport='image_raw'):
    """
    Returns the number of camera topics
    """
    topics = find_camera_image_topics(transport=transport)
    return len(topics)

def camera_nodes_ready(mode='inspector'):
    """
    Checks to see if the camera nodes are ready. The camera nodes are
    considered ready when the number of camera topics (with transport
    raw_image) is equal to the number of cameras expected.

    The number of cameras expected depends on the mode. If mode is
    equal to 'inspector' then the number of cameras is equal to the number
    of cameras found attached to the system. If mode is anything else then
    the number of cameras is equal that given in the camera assignment file.

    """
    mode = mode.lower()
    if mode == 'inspector':
        number_of_cameras = get_number_of_cameras()
    else:
        camera_assignment = get_camera_assignment()
        number_of_cameras = len(camera_assignment)
        
    number_of_camera_image_topics = get_number_of_camera_image_topics()
    if number_of_cameras == number_of_camera_image_topics:
        return True
    else:
        return False

def get_machine_def():
    """
    Reads the machine definition file.
    """
    return file_tools.read_machine_def()

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

def get_camera_assignment():
    """
    Reads the current camera assignment yaml file.
    """
    return file_tools.read_camera_assignment()

def get_frame_rates():
    """
    Reads the frame_rates.yaml file and returns a dictionary of the allowed frame
    rates.
    """
    return file_tools.read_frame_rates()

def get_camera_calibrator_nodes():
    """
    Returns a list of the currently running camera calibrator nodes
    """
    node_list = get_nodes()
    calibrator_nodes = [node for node in node_list if 'camera_calibrator' in node]
    return calibrator_nodes

def get_camera_calibrate_services():
    """
    Returns a list of the camera calibrate services.
    """
    srv_list = get_services()
    cal_list = [srv for srv in srv_list if ('calibrate' in srv) and  ('camera' in srv)]
    return cal_list


def get_camera_calibration_info():
    """
    Returns a dictionary of information regarding the existing camera calibration
    files.
    """
    cal_files = file_tools.get_camera_calibration_files(fullpath=True)
    return get_cal_files_info(cal_files)

def get_homography_calibration_info():
    """
    Returns a dictionary of information regarding the existing homography calibraiton 
    files.
    """
    cal_files = file_tools.get_homography_calibration_files(fullpath=True)
    return get_cal_files_info(cal_files)

def get_transform_2d_calibration_info():
    """
    Returns a dictionary of information regarding the existing transform 2d calibratiion
    files.
    """
    cal_files = file_tools.get_transform_2d_calibration_files(fullpath=True)
    return get_cal_files_info(cal_files)

def get_cal_files_info(cal_files):
    """
    Returns a dictionary of information regarding the list of given calibratoin files.
    """
    cal_info = {}
    for f in cal_files:
        f_name = os.path.split(f)[-1]
        base_name, dummy = os.path.splitext(f_name)
        f_time = file_tools.get_last_modified_time(f)
        cal_info[base_name] = {'file': f, 'modified': f_time}
    return cal_info

def get_camera_namespace_dict():
    """
    Returns a dictionary of the camera namespaces. 
    """
    image_topic_list = find_camera_image_topics(transport='image_raw') 
    namespace_dict = {}
    for image_topic in image_topic_list:
        image_topic_split = image_topic.split('/')
        camera = image_topic_split[2]
        namespace = '/'.join(image_topic_split[:4])
        namespace_dict[camera] = namespace
    return namespace_dict

def get_calibrated_cameras():
    """
    Returns a list of the cameras for which there exist calibration
    files.
    """
    cal_file_list = file_tools.get_camera_calibration_files(fullpath=False)
    cam_list = []
    for cal_file in cal_file_list:
        cam, dummy = os.path.splitext(cal_file)
        cam_list.append(cam)
    return cam_list

def get_image_proc_nodes():
    """
    Returns a list of the running image_proc nodes
    """
    node_list = get_nodes()
    image_proc_list = []
    for node in node_list:
        if ('image_proc' in node) and (not 'image_proc_master' in node):
            image_proc_list.append(node)
    return image_proc_list

def image_proc_nodes_ready():
    """
    Returns true is the image proc nodes are ready. The image_proc nodes are
    considered to be ready when the number of image_proc nodes is equal to the
    number of calibrated cameras.  
    """
    # Get number of calibrated cameras
    calibrated_list = get_calibrated_cameras()
    num_calibrated = len(calibrated_list)

    # Get number of image_proc nodes
    image_proc_list = get_image_proc_nodes()
    num_image_proc = len(image_proc_list)

    if num_image_proc == num_calibrated:
        return True
    else:
        return False

def image_rect_ready():
    """
    Returns true is the number of image_rect topics is equal to the number of image_proc
    nodes
    """
    # Get number of image_rect topics
    image_rect_list = find_camera_image_topics(transport='image_rect')
    num_image_rect = len(image_rect_list)

    # Get number of image_proc nodes 
    image_proc_list = get_image_proc_nodes()
    num_image_proc = len(image_proc_list)
    
    if num_image_rect == num_image_proc:
        return True
    else:
        return False

def zoom_tool_image_ready():
    """
    Returns true if the number of zoom tool images is equal to the number of image_raw
    images.
    """
    image_raw_list = find_camera_image_topics(transport='image_raw')
    zoom_tool_list = find_camera_image_topics(transport='image_zoom_tool')
    if len(zoom_tool_list) == len(image_raw_list):
        return True
    else:
        return False

def get_homography_calibrator_nodes():
    """
    Returns a list of the homography calibrator nodes.
    """
    node_list = get_nodes()
    calibrator_nodes = []
    for node in node_list:
        if ('homography_calibrator' in node) and (not 'homography_calibrator_master' in node):
            calibrator_nodes.append(node)
    return calibrator_nodes

def homography_calibrator_nodes_ready():
    """
    Returns true if the homography calibrator nodes are ready. They nodes are 
    considered ready if the number of homography calibrator nodes is equal to the 
    number of image_proc nodes
    """
    # Get number of image_proc nodes
    image_proc_list = get_image_proc_nodes()
    num_image_proc = len(image_proc_list)

    # Get number of homography calibrator nodes
    calibrator_list = get_homography_calibrator_nodes()
    num_calibrator = len(calibrator_list)

    if num_image_proc == num_calibrator:
        return True
    else:
        return False

def get_transform_2d_calibrator_nodes():
    """
    Returns a list of the transform 2d calibrator nodes.
    """
    node_list = get_nodes()
    calibrator_nodes = []
    for node in node_list:
        if ('transform_2d_calibrator' in node) and (not 'transform_2d_calibrator_master' in node):
            calibrator_nodes.append(node)
    return calibrator_nodes

def transform_2d_calibrator_nodes_ready():
    """
    Returns True if the transform 2d calibrator nodes are ready. They are ready 
    if the number of transform 2d calibrator nodes are eqaul to the number of
    camera pairs.
    """
    # Get list of currently running transform 2d calibrators
    calibrator_list = get_transform_2d_calibrator_nodes()

    # Get list of all camera pairs
    camera_pairs_dict = file_tools.read_tracking_2d_camera_pairs()
    all_pairs_list = []
    for pairs_list in camera_pairs_dict.values():
        all_pairs_list.extend(pairs_list)

    if len(all_pairs_list) == len(calibrator_list):
        return True
    else:
        return False


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    

    if 0:
        node_list = get_nodes()
        print('node_list:')
        print(node_list)
        print()

    if 0:
        node_list = get_nodes()
        node_list.append('/bobs_node')
        for node in node_list:
            flag = exists_node(node)
            print('extis_node({0}) = {1}'.format(node,flag))
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
        topic_list = find_camera_image_topics()
        print('camera_image_topics:')
        for topic in topic_list:
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

    if 0:
        num = get_number_of_camera_image_topics()
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

    if 0:
        camera_assignment = get_camera_assignment()
        for k,v in camera_assignment.iteritems():
            print(k)
            for kk, vv in v.iteritems():
                print('  ',kk,vv)
    if 0:

        frame_rates = get_frame_rates()
        for k,v in frame_rates.iteritems():
            print(k,v)

    if 0:

        node_list = get_camera_calibrator_nodes()
        for node in node_list:
            print(node)

    if 0:
        srv_list = get_camera_calibrate_services()
        print(srv_list)


    if 0:
        cal_info = get_camera_calibration_info()
        print(cal_info)

    if 0:
        cal_info = get_homography_calibration_info()
        print(cal_info)

    if 1:
        cal_info = get_transform_2d_calibration_info()
        print(cal_info)

    if 0:
        namespace_dict = get_camera_namespace_dict()
        for k,v in namespace_dict.iteritems():
            print(k,v)

    if 0:
        camera_list = get_calibrated_cameras()
        print(camera_list)

    if 0:
        node_list = get_image_proc_nodes()
        print(node_list)

    if 0:
        flag = image_proc_nodes_ready()
        print(flag)

    if 0:
        node_list = get_homography_calibrator_nodes()
        print(node_list)

    if 0:
        flag = homography_calibrator_nodes_ready()
        print(flag)

    if 0:
        flag = image_rect_ready()
        print(flag)

    if 0:
        flag = zoom_tool_ready()
        print(flag)

    if 0:
        node_list = get_transform_2d_calibrator_nodes()
        print(node_list)

    if 0:
        flag = transform_2d_calibrator_nodes_ready()
        print(flag)

    if 0:
        topic_list = find_topics_w_ending('image_raw')
        print(len(topic_list))
        print(topic_list)
        topic_list = find_topics_w_ending('image_rect')
        print(len(topic_list))
        print(topic_list)



            
