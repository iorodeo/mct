import roslib
roslib.load_manifest('mct_xml_tools')
import rospy
import os
import os.path
import jinja2
import yaml
import mct_introspection

def create_inspector_launch(filename,machine_names):
    """
    Creates launch file for camera inspector nodes.
    """
    file_path, file_name = os.path.split(__file__)
    template_dir = os.path.join(file_path, 'templates')
    template_name ='inspector_nodes_launch.xml'
    machine_file = os.path.join(os.environ['MCT_CONFIG'],'machine','mct.machine')

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
    file_path, file_name = os.path.split(__file__)
    template_dir = os.path.join(file_path, 'templates')
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
    file_path, file_name = os.path.split(__file__)
    template_dir = os.path.join(file_path, 'templates')
    template_name = 'inspector_camera_launch.xml'
    machine_file = os.path.join(os.environ['MCT_CONFIG'],'machine','mct.machine')

    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(machine_file=machine_file, camera_dict=camera_dict)

    with open(filename,'w') as f:
        f.write(xml_str)

def create_mjpeg_server_launch(filename, mjpeg_info_dict):
    """
    Creates a launch file for the mjpeg servers based on the mjpeg information
    dictionary, mjpeg_info_dict. This function is designed to be called from
    the mjpeg_manager node.
    """
    file_path, file_name = os.path.split(__file__)
    template_dir = os.path.join(file_path, 'templates')
    template_name = 'mjpeg_server_launch.xml'
    machine_file = os.path.join(os.environ['MCT_CONFIG'],'machine','mct.machine')
    
    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(machine_file=machine_file, mjpeg_info_dict=mjpeg_info_dict)

    with open(filename,'w') as f:
        f.write(xml_str)

def create_camera_launch(filename,camera_assignment,trigger=False):
    """
    Generates a camera launch file based on the current camera assignment.

    Note, assumes that the camera yaml files have been added to the camera
    assignment.
    """
    file_path, file_name = os.path.split(__file__)
    template_dir = os.path.join(file_path, 'templates')
    template_name = 'camera_launch.xml'
    machine_file = os.path.join(os.environ['MCT_CONFIG'],'machine','mct.machine')

    jinja2_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir))
    template = jinja2_env.get_template(template_name)
    xml_str = template.render(
            machine_file=machine_file,
            camera_assignment=camera_assignment,
            trigger=trigger,
            )

    with open(filename,'w') as f:
        f.write(xml_str)

def create_camera_yaml(directory,camera_assignment):
    """
    Creates the yaml files for the given camera assignment.

    Also, adds the yaml file to the information dictionary for each camera in
    the assignment.
    """
    for camera, info in camera_assignment.iteritems():
        filename = os.path.join(directory, '{0}.yaml'.format(camera))
        info['yaml_file'] = filename
        with open(filename,'w') as f:
            data = {'guid': info['guid'], 'frame_id': camera}
            yaml.dump(data,f,default_flow_style=False)
    return camera_assignment

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
                '00305300013f2ef3': {'camera_topic': '/mct_slave1/00305300013f2ef3/camera/image_raw', 'mjpeg_port': 8083, 'mjpeg_server': 'mjpeg_server_00305300013f2ef3'}, 
                '00305300013f2ef6': {'camera_topic': '/mct_slave1/00305300013f2ef6/camera/image_raw', 'mjpeg_port': 8085, 'mjpeg_server': 'mjpeg_server_00305300013f2ef6'}, 
                '00305300013f2ef7': {'camera_topic': '/mct_slave2/00305300013f2ef7/camera/image_raw', 'mjpeg_port': 8089, 'mjpeg_server': 'mjpeg_server_00305300013f2ef7'}, 
                '00305300013f2ef4': {'camera_topic': '/mct_master/00305300013f2ef4/camera/image_raw', 'mjpeg_port': 8088, 'mjpeg_server': 'mjpeg_server_00305300013f2ef4'}, 
                '00305300013f2ef5': {'camera_topic': '/mct_master/00305300013f2ef5/camera/image_raw', 'mjpeg_port': 8082, 'mjpeg_server': 'mjpeg_server_00305300013f2ef5'}, 
                '0030530001410997': {'camera_topic': '/mct_slave1/0030530001410997/camera/image_raw', 'mjpeg_port': 8081, 'mjpeg_server': 'mjpeg_server_0030530001410997'}, 
                '00305300013f2ef8': {'camera_topic': '/mct_slave2/00305300013f2ef8/camera/image_raw', 'mjpeg_port': 8084, 'mjpeg_server': 'mjpeg_server_00305300013f2ef8'}, 
                '00305300013f2ef9': {'camera_topic': '/mct_slave2/00305300013f2ef9/camera/image_raw', 'mjpeg_port': 8080, 'mjpeg_server': 'mjpeg_server_00305300013f2ef9'}, 
                '00305300013f2efb': {'camera_topic': '/mct_master/00305300013f2efb/camera/image_raw', 'mjpeg_port': 8090, 'mjpeg_server': 'mjpeg_server_00305300013f2efb'}, 
                '00305300013f2efa': {'camera_topic': '/mct_master/00305300013f2efa/camera/image_raw', 'mjpeg_port': 8087, 'mjpeg_server': 'mjpeg_server_00305300013f2efa'}, 
                '003053000140e715': {'camera_topic': '/mct_slave1/003053000140e715/camera/image_raw', 'mjpeg_port': 8086, 'mjpeg_server': 'mjpeg_server_003053000140e715'}, 
                '0030530001412079': {'camera_topic': '/mct_slave2/0030530001412079/camera/image_raw', 'mjpeg_port': 8091, 'mjpeg_server': 'mjpeg_server_0030530001412079'}
                }
        create_mjpeg_server_launch(filename,mjpeg_info_dict)

    if 1:
        filename = 'camera.launch'
        yaml_directory = './'
        
        camera_assignment = mct_introspection.get_camera_assignment()
        create_camera_yaml(yaml_directory,camera_assignment)
        create_camera_launch(filename,camera_assignment)
       


