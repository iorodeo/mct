from __future__ import print_function
import roslib
roslib.load_manifest('mct_xml_tools')
import rospy
import os
import os.path
import jinja2
import yaml
import mct_introspection
import mct_utilities

file_path, dummy = os.path.split(__file__)
template_dir = os.path.join(file_path, 'templates')

def create_inspector_launch(filename,machine_names):
    """
    Creates launch file for camera inspector nodes.
    """
    template_name ='inspector_nodes_launch.xml'
    machine_file = mct_utilities.file_tools.machine_launch_file

    # Use jinja2 to create xml string
    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(machine_file=machine_file, machine_names=machine_names)

    # Write lauch file
    with open(filename,'w') as f:
        f.write(xml_str)

def create_machine_launch(filename,machine_def):
    """
    Creates the mct_machine.launch file from the machine definition found in
    the machine_def.yaml file.
    """
    template_name = 'mct_machine.xml'
    user = machine_def['user']
    master_info = machine_def['mct_master']
    master_info['name'] = 'mct_master'
    master_info['default'] = 'true'

    slave_keys = machine_def.keys()
    slave_keys.remove('mct_master')
    slave_keys.remove('user')
    slave_keys.sort()
    machine_info_list = [master_info]
    for name in slave_keys:
        slave_info = machine_def[name]
        slave_info['name' ] = name
        slave_info['default'] = 'false'
        machine_info_list.append(slave_info)

    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(user=user,machine_info_list=machine_info_list)

    with open(filename, 'w') as f:
        f.write(xml_str)

def create_inspector_camera_yaml(tmp_dir,camera_dict):
    """
    Creates the yaml files for each camera which are required for the 
    lauch file. Note, add the yaml file to the info dictionary for each 
    camera for later use.
    """
    for guid, info in camera_dict.iteritems():
        filename = os.path.join(tmp_dir,'camera_{0}.yaml'.format(guid))
        info['yaml_file'] = filename
        with open(filename,'w') as f:
            data = {'guid': guid, 'frame_id': 'camera_{0}'.format(guid)}
            yaml.dump(data,f,default_flow_style=False)
    return camera_dict

def create_inspector_camera_launch(filename, camera_dict):
    """
    Creates camera launch file which will called from the camera inspector node.

    Note, assumes that the yaml files for the cameras have been added to the
    info dict for each camera.
    """
    template_name = 'inspector_camera_launch.xml'
    machine_file = mct_utilities.file_tools.machine_launch_file
    frame_rate_dict = mct_introspection.get_frame_rates()
    frame_rate = frame_rate_dict['assignment']

    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(
            machine_file=machine_file, 
            camera_dict=camera_dict,
            frame_rate=frame_rate,
            )

    with open(filename,'w') as f:
        f.write(xml_str)

def create_mjpeg_server_launch(filename, mjpeg_info_dict):
    """
    Creates a launch file for the mjpeg servers based on the mjpeg information
    dictionary, mjpeg_info_dict. This function is designed to be called from
    the mjpeg_manager node.
    """
    template_name = 'mjpeg_server_launch.xml'
    machine_file = mct_utilities.file_tools.machine_launch_file
    
    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(machine_file=machine_file, mjpeg_info_dict=mjpeg_info_dict)

    with open(filename,'w') as f:
        f.write(xml_str)

def create_camera_launch(filename, camera_assignment, frame_rate='default', trigger=False):
    """
    Generates a camera launch file based on the current camera assignment.

    Note, if camera assignment is given it assumes that the camera yaml files
    have been added to the camera assignment.
    """
    template_name = 'camera_launch.xml'
    machine_file = mct_utilities.file_tools.machine_launch_file
    frame_rate_dict = mct_introspection.get_frame_rates()
    frame_rate_value = frame_rate_dict[frame_rate]

    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(
            machine_file=machine_file,
            camera_assignment=camera_assignment,
            trigger=trigger,
            frame_rate=frame_rate_value,
            )

    with open(filename,'w') as f:
        f.write(xml_str)

def create_camera_yaml(directory, camera_assignment):
    """
    Creates the yaml files for the given camera assignment.

    Also, adds the yaml file to the information dictionary for each camera in
    the assignment.
    """
    for camera, info in camera_assignment.iteritems():
        filename = os.path.join(directory, '{0}.yaml'.format(camera))
        info['yaml_file'] = filename
        with open(filename,'w') as f:
            f.write('\n# Autogenerated configuration file - do not hand edit\n\n')
            data = {'guid': info['guid'], 'frame_id': camera}
            yaml.dump(data,f,default_flow_style=False)
    return camera_assignment

def create_camera_calibrator_launch(filename, image_topics, chessboard_size, chessboard_square):
    """
    Create launch file for camera calibrators.
    """
    template_name = 'camera_calibrator_launch.xml'
    machine_file = mct_utilities.file_tools.machine_launch_file

    # Pack up calibrator data
    camera_names = [val.split('/')[2] for val in image_topics]
    camera_topics = [val.replace('/image_raw','') for val in image_topics]
    machines = [val.split('/')[1] for val in image_topics]
    calibrator_data = zip(camera_names, camera_topics, image_topics, machines)

    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(
            machine_file=machine_file,
            calibrator_data=calibrator_data,
            chessboard_size=chessboard_size,
            chessboard_square=chessboard_square,
            )

    with open(filename,'w') as f:
        f.write(xml_str)

def create_image_proc_launch(filename):
    """
    Creates launch file for image rectification and processing nodes
    """
    template_name = 'image_proc_launch.xml'
    machine_file = mct_utilities.file_tools.machine_launch_file

    # Create dictionary which associates a camera name to its namespace for all
    # cameras which have a calibration   
    namespace_dict = mct_introspection.get_camera_namespace_dict()
    camera_list = mct_introspection.get_calibrated_cameras()
    for camera in namespace_dict.keys():
        if not camera in camera_list:
            namespace_dict.pop(camera)

    # Create a list given pairs (camera namespace, computer) on which to launch
    # image_proc nodes.
    camera_assignment = mct_introspection.get_camera_assignment()
    launch_list = []
    for camera, namespace in namespace_dict.iteritems():
        computer = camera_assignment[camera]['computer']
        launch_list.append((namespace,computer))

    # Create xml launch file
    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(machine_file=machine_file, launch_list=launch_list)

    with open(filename,'w') as f:
        f.write(xml_str)

def create_homography_calibrator_launch(filename):
    """
    Creates launch file for homography calibrators
    """
    template_name = 'homography_calibrator_launch.xml'
    machine_file = mct_utilities.file_tools.machine_launch_file
    params_file = mct_utilities.file_tools.homography_calibrator_params_file

    # Get list of pairs (namespace, rectified images)
    image_rect_list = mct_introspection.find_camera_image_topics(transport='image_rect')
    launch_list = []
    for topic in image_rect_list:
        print(topic)
        topic_split = topic.split('/')
        namespace = '/'.join(topic_split[:4])
        launch_list.append((namespace,topic))

    # Create xml launch file
    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(
            machine_file=machine_file, 
            params_file=params_file,
            launch_list=launch_list
            )
    with open(filename,'w') as f:
        f.write(xml_str)

def create_zoom_tool_launch(filename):
    """
    Creates launch file for zoom tool nodes.
    """
    template_name = 'zoom_tool_launch.xml'
    machine_file = mct_utilities.file_tools.machine_launch_file
    params_file = mct_utilities.file_tools.zoom_tool_params_file
    camera_assignment = mct_utilities.file_tools.read_camera_assignment()
    
    # Create launch list for zoom tools (namespace, topic, computer)
    image_topics = mct_introspection.find_camera_image_topics(transport='image_raw')
    launch_list = []
    for topic in image_topics:
        topic_split = topic.split('/')
        namespace = '/'.join(topic_split[:4])
        camera = topic_split[2]
        computer = camera_assignment[camera]['computer']
        launch_list.append((namespace,topic,computer))

    # Create xml launch file
    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(
            machine_file=machine_file, 
            params_file=params_file,
            launch_list=launch_list
            )
    with open(filename,'w') as f:
        f.write(xml_str)

def create_transform_2d_calibrator_launch(filename):
    """
    Creates launch file for transform_2d_calibrator nodes.
    """
    template_name = 'transform_2d_calibrator_launch.xml'
    machine_file = mct_utilities.file_tools.machine_launch_file
    params_file = mct_utilities.file_tools.transform_2d_calibrator_params_file

    # Get region and camera pair information
    region_dict = mct_utilities.file_tools.read_tracking_2d_regions()
    camera_pairs_dict = mct_utilities.file_tools.read_tracking_2d_camera_pairs() 

    # Get dictionary of camera to rectified images
    image_rect_list = mct_introspection.find_camera_image_topics(transport='image_rect')
    camera_to_image_rect = {}
    for camera_list in region_dict.values():
        for camera in camera_list:
            for image_rect in image_rect_list:
                if camera in image_rect.split('/'):
                    camera_to_image_rect[camera] = image_rect

    # Create launch list (namespace, topic0, topic1)
    launch_list = []
    for pairs_list in camera_pairs_dict.values():
        for camera0, camera1 in pairs_list:
            try:
                namespace = '/{0}_{1}'.format(camera0, camera1)
                topic0 = camera_to_image_rect[camera0]
                topic1 = camera_to_image_rect[camera1]
                launch_list.append((namespace, topic0, topic1))
            except KeyError:
                pass
    
    # Create xml launch file
    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(
            machine_file=machine_file, 
            params_file=params_file,
            launch_list=launch_list
            )
    with open(filename,'w') as f:
        f.write(xml_str)
            

def create_static_tf_publisher_2d_launch(filename):
    """
    Create static transform publisher launch file.

    This is not currently used.
    """
    template_name = 'static_tf_publisher_2d_launch.xml'
    machine_file = mct_utilities.file_tools.machine_launch_file

    # Generate lauch list (frame_0, frame_1
    launch_list = []
    camera_pairs_dict = mct_utilities.file_tools.read_tracking_2d_camera_pairs()
    for pairs_list in camera_pairs_dict.values():
        for cam_0, cam_1 in pairs_list:
            # Get camera numbers and use to form frame ids for tracking planes
            num_0 = int(cam_0.split('_')[1])
            num_1 = int(cam_1.split('_')[1])
            frame_0 = 'tracking_plane_{0}'.format(num_0)
            frame_1 = 'tracking_plane_{0}'.format(num_1)

            # Get 2d transformation and make 3d
            transform_2d = mct_utilities.file_tools.read_transform_2d_calibration(cam_0,cam_1)
            tx = transform_2d['translation_x']
            ty = transform_2d['translation_y']
            tz = 0.0
            ang = transform_2d['rotation']
            rate_ms = 10
            node_name = 'tracking_plane_{0}_to_{1}_broadcaster'.format(num_0,num_1)
            node_args = '{0} {1} {2} {3} {4} {5} {6} {7} {8}'.format(tx, ty, tz, 0.0, 0.0, ang, frame_0, frame_1, rate_ms)
            launch_list.append((node_name,node_args))

    # Create xml launch file
    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(
            machine_file=machine_file, 
            launch_list=launch_list
            )

    with open(filename,'w') as f:
        f.write(xml_str)


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Testing and development

    if 0:
        filename = 'camera1394_inspector.launch'
        machines = ['c1', 'c2', 'c3']
        create_inspector_launch(filename,machines)

    if 0:
        filename = 'mct.mcahine'
        machine_def = {
                'user' : 'albert',
                'mct_master' : {'address' : 'felis'},
                }
        for i in range(0,10):
            machine_def['mct_slave{0}'.format(i)] = {'address' : 'tabby{0}'.format(i)}
        create_machine_launch(filename,machine_def)

    if 0:
        filename = 'inspector_camera.launch'
        tmp_dir = '.'
        camera_dict = {
                '30530001412079' :  {'machine': 'slave2', 'model': 'scA640-120fm', 'vendor': 'Basler', 'unit': 0},
                '305300013f2efa' :  {'machine': 'master', 'model': 'scA640-120fm', 'vendor': 'Basler', 'unit': 0},
                '305300013f2efb' :  {'machine': 'master', 'model': 'scA640-120fm', 'vendor': 'Basler', 'unit': 0},
                '305300013f2ef9' :  {'machine': 'slave2', 'model': 'scA640-120fm', 'vendor': 'Basler', 'unit': 0},
                '3053000140e715' :  {'machine': 'slave1', 'model': 'scA640-120fm', 'vendor': 'Basler', 'unit': 0},
                '305300013f2ef8' :  {'machine': 'slave2', 'model': 'scA640-120fm', 'vendor': 'Basler', 'unit': 0},
                '30530001410997' :  {'machine': 'slave1', 'model': 'scA640-120fm', 'vendor': 'Basler', 'unit': 0},
                '305300013f2ef4' :  {'machine': 'master', 'model': 'scA640-120fm', 'vendor': 'Basler', 'unit': 0},
                '305300013f2ef5' :  {'machine': 'master', 'model': 'scA640-120fm', 'vendor': 'Basler', 'unit': 0},
                '305300013f2ef6' :  {'machine': 'slave1', 'model': 'scA640-120fm', 'vendor': 'Basler', 'unit': 0},
                '305300013f2ef7' :  {'machine': 'slave2', 'model': 'scA640-120fm', 'vendor': 'Basler', 'unit': 0},
                '305300013f2ef3' :  {'machine': 'slave1', 'model': 'scA640-120fm', 'vendor': 'Basler', 'unit': 0},
                }
        create_inspector_camera_yaml(tmp_dir,camera_dict)
        create_inspector_camera_launch(filename, camera_dict)

    if 0:
        filename = 'mjpeg_server.launch'
        mjpeg_info_dict = {
                '00305300013f2ef3': {'image_topic': '/mct_slave1/00305300013f2ef3/camera/image_raw', 'mjpeg_port': 8083, 'mjpeg_server': 'mjpeg_server_00305300013f2ef3'}, 
                '00305300013f2ef6': {'image_topic': '/mct_slave1/00305300013f2ef6/camera/image_raw', 'mjpeg_port': 8085, 'mjpeg_server': 'mjpeg_server_00305300013f2ef6'}, 
                '00305300013f2ef7': {'image_topic': '/mct_slave2/00305300013f2ef7/camera/image_raw', 'mjpeg_port': 8089, 'mjpeg_server': 'mjpeg_server_00305300013f2ef7'}, 
                '00305300013f2ef4': {'image_topic': '/mct_master/00305300013f2ef4/camera/image_raw', 'mjpeg_port': 8088, 'mjpeg_server': 'mjpeg_server_00305300013f2ef4'}, 
                '00305300013f2ef5': {'image_topic': '/mct_master/00305300013f2ef5/camera/image_raw', 'mjpeg_port': 8082, 'mjpeg_server': 'mjpeg_server_00305300013f2ef5'}, 
                '0030530001410997': {'image_topic': '/mct_slave1/0030530001410997/camera/image_raw', 'mjpeg_port': 8081, 'mjpeg_server': 'mjpeg_server_0030530001410997'}, 
                '00305300013f2ef8': {'image_topic': '/mct_slave2/00305300013f2ef8/camera/image_raw', 'mjpeg_port': 8084, 'mjpeg_server': 'mjpeg_server_00305300013f2ef8'}, 
                '00305300013f2ef9': {'image_topic': '/mct_slave2/00305300013f2ef9/camera/image_raw', 'mjpeg_port': 8080, 'mjpeg_server': 'mjpeg_server_00305300013f2ef9'}, 
                '00305300013f2efb': {'image_topic': '/mct_master/00305300013f2efb/camera/image_raw', 'mjpeg_port': 8090, 'mjpeg_server': 'mjpeg_server_00305300013f2efb'}, 
                '00305300013f2efa': {'image_topic': '/mct_master/00305300013f2efa/camera/image_raw', 'mjpeg_port': 8087, 'mjpeg_server': 'mjpeg_server_00305300013f2efa'}, 
                '003053000140e715': {'image_topic': '/mct_slave1/003053000140e715/camera/image_raw', 'mjpeg_port': 8086, 'mjpeg_server': 'mjpeg_server_003053000140e715'}, 
                '0030530001412079': {'image_topic': '/mct_slave2/0030530001412079/camera/image_raw', 'mjpeg_port': 8091, 'mjpeg_server': 'mjpeg_server_0030530001412079'}
                }
        create_mjpeg_server_launch(filename,mjpeg_info_dict)

    if 0:
        filename = 'camera.launch'
        yaml_directory = './'
        camera_assignment = mct_introspection.get_camera_assignment()
        create_camera_yaml(directory=yaml_directory,camera_assignment=camera_assignment)
        create_camera_launch(filename=filename,camera_assignment=camera_assignment)

    if 0:
        filename = 'camera_calibrator.launch'
        image_topics = [
                '/mct_slave2/camera_10/camera/image_raw', 
                '/mct_slave2/camera_12/camera/image_raw', 
                '/mct_slave1/camera_9/camera/image_raw', 
                '/mct_slave2/camera_7/camera/image_raw', 
                '/mct_slave2/camera_5/camera/image_raw', 
                '/mct_master/camera_8/camera/image_raw', 
                '/mct_master/camera_6/camera/image_raw', 
                '/mct_slave1/camera_1/camera/image_raw', 
                '/mct_slave1/camera_2/camera/image_raw', 
                '/mct_master/camera_11/camera/image_raw', 
                '/mct_master/camera_3/camera/image_raw', 
                '/mct_slave1/camera_4/camera/image_raw'
                ]
        chessboard_size = '8x6'
        chessboard_square = '0.0254'
        create_camera_calibrator_launch(filename,image_topics,chessboard_size,chessboard_square)

    if 0:
        filename = 'image_proc.launch'
        create_image_proc_launch(filename)

    if 0:
        filename = 'homography_calibrator.launch'
        create_homography_calibrator_launch(filename)

    if 0:
        filename = 'zoom_tool.launch'
        create_zoom_tool_launch(filename)

    if 0:
        filename = 'transform_2d_calibrator.launch'
        create_transform_2d_calibrator_launch(filename)

    if 1:
        filename = 'static_tf_publisher_2d.launch'
        create_static_tf_publisher_2d_launch(filename)


       


