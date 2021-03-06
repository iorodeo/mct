from __future__ import print_function
import os
import os.path
import yaml
import subprocess
import time
import shutil

# Configuration directories
config_dir = os.environ['MCT_CONFIG']
cameras_dir = os.path.join(config_dir,'cameras')
camera_calibration_dir = os.path.join(cameras_dir, 'calibrations')
camera_parameters_dir = os.path.join(cameras_dir, 'parameters')
machine_dir = os.path.join(config_dir,'machine')
targets_dir = os.path.join(config_dir, 'targets')
tracking_2d_dir = os.path.join(config_dir, 'tracking_2d')
homographies_dir = os.path.join(tracking_2d_dir, 'homographies')
transforms_2d_dir = os.path.join(tracking_2d_dir,'transforms')
ros_camera_info_dir = os.path.join(os.environ['HOME'],'.ros', 'camera_info')
lighting_dir = os.path.join(config_dir, 'lighting')
logging_dir = os.path.join(config_dir, 'logging')

# Configuration files
camera_assignment_file  = os.path.join(config_dir, 'cameras', 'camera_assignment.yaml')
frame_rates_file = os.path.join(cameras_dir, 'frame_rates.yaml')
machine_def_file = os.path.join(machine_dir,'machine_def.yaml')
machine_launch_file = os.path.join(machine_dir,'mct.machine')
homography_calibrator_params_file = os.path.join(homographies_dir, 'calibrator_params.yaml')
transform_2d_calibrator_params_file = os.path.join(transforms_2d_dir, 'calibrator_params.yaml')
zoom_tool_params_file = os.path.join(cameras_dir,'zoom_tool_params.yaml')
tracking_2d_regions_file = os.path.join(tracking_2d_dir,'regions.yaml')
tracking_2d_camera_pairs_file = os.path.join(tracking_2d_dir,'camera_pairs.yaml')
tracking_2d_stitching_params_file = os.path.join(tracking_2d_dir,'stitching_params.yaml')
lighting_mightex_params_file = os.path.join(lighting_dir, 'mightex_controllers.yaml')
logging_params_file = os.path.join(logging_dir, 'logging_params.yaml')
logging_extra_video_file = os.path.join(logging_dir, 'extra_video.yaml')


def read_yaml_file(filename):
    """
    Reads the given yaml file and returns a dictionary with its contents
    """
    with open(filename,'r') as f:
        yaml_dict = yaml.load(f)
        if yaml_dict is None:
            yaml_dict = {}
    return yaml_dict

def write_yaml_file(filename,yaml_dict):
    """
    Writes the given dictionary to the specified yaml file.
    """
    with open(filename,'w') as f:
        yaml.dump(yaml_dict, f, default_flow_style=False)

def read_logging_params():
    """
    Reads the logging parameters file.
    """
    return read_yaml_file(logging_params_file)

def read_logging_extra_video():
    """
    Reads the logging parameters extra video file.
    """
    return read_yaml_file(logging_extra_video_file)

def read_mightex_params():
    """
    Reads the mightex controller parameters from the file in the  lighting directory.
    """
    return read_yaml_file(lighting_mightex_params_file)

def write_mightex_params(params):
    """
    writes the mightex controller parameters to the file in the lighting directory.
    """
    try:
        curr_params = read_mightex_params()
    except IOError:
        curr_params = {}
    curr_params.update(params)
    write_yaml_file(lighting_mightex_params_file, params)

def write_camera_params(camera, params):
    """
    Write camera parameters to file
    """
    filename = os.path.join(camera_parameters_dir,'{0}.yaml'.format(camera))
    write_yaml_file(filename,params)

def read_camera_params(camera):
    """
    Read camera parameters from file.
    """
    filename = os.path.join(camera_parameters_dir,'{0}.yaml'.format(camera))
    return read_yaml_file(filename)
    
def read_tracking_2d_stitching_params():
    """
    Reads the stitching_params.yaml file from the tracking 2d directory.
    """
    return read_yaml_file(tracking_2d_stitching_params_file)

def read_tracking_2d_regions():
    """
    Reads the regions.yaml file from the tracking 2d directory
    """
    return read_yaml_file(tracking_2d_regions_file)

def read_tracking_2d_camera_pairs():
    """
    Reads the camera_pairs file from the tracking 2d directory
    """
    return read_yaml_file(tracking_2d_camera_pairs_file)

def read_machine_def():
    """
    Reads the machine definition file.
    """
    return read_yaml_file(machine_def_file)

def write_camera_assignment(yaml_dict):
    """
    Write camera assignment to mct configuration directory
    """
    # Write yaml file to 'camera_assignment.yaml' in cameras section of mct configuration
    with open(camera_assignment_file,'w') as f:
        f.write('\n# Autogenerated configuration file - do not hand edit\n\n')
        camera_name_list = yaml_dict.keys()
        camera_name_list.sort
        yaml.dump(yaml_dict,f,default_flow_style=False)

def read_camera_assignment():
    """
    Reads the current camera assignment from the camera assignment yaml file. If 
    this file doesn't exist then None is returned.
    """
    if os.path.isfile(camera_assignment_file):
        return read_yaml_file(camera_assignment_file)
    else:
        return None

def read_frame_rates():
    """
    Reads the frame_rates.yaml file and returns a dictionary of the allowed frame
    rates.
    """
    return read_yaml_file(frame_rates_file)

def get_camera_calibration_files(fullpath=True):
    """
    Returns a list of the camera calibration yaml files in the calibration directory of
    the mct configuration.
    """
    file_list = os.listdir(camera_calibration_dir)
    file_list = [f for f in file_list if 'camera_' in f]
    if fullpath:
        file_list = [os.path.join(camera_calibration_dir,f) for f in file_list]
    return file_list

def get_homography_calibration_files(fullpath=True):
    """
    Returns a list of the homography calibration yaml files in the homgraphies directory
    of the mct configuration.
    """
    file_list = os.listdir(homographies_dir)
    dummy, params_file = os.path.split(homography_calibrator_params_file)
    file_list.remove(params_file)
    if fullpath:
        file_list = [os.path.join(homographies_dir,f) for f in file_list]
    return file_list

def write_camera_calibration(camera_name, cal_ost_str):
    """
    Writes the camera calibration given the camera name and calibration ost
    string produced by the cameracalibrator node.

    This process is a bit convoluted - but I want to stick with the standard
    ROS tools while still adding the "Autogenerated" tag to the top of the yaml
    files.
    """
    if not os.path.isdir(camera_calibration_dir):
        os.mkdir(camera_calibration_dir)
    basename = os.path.join(camera_calibration_dir, camera_name)
    filename_ost = '{0}.ini'.format(basename)
    filename_yaml = '{0}.yaml'.format(basename)

    # Write - ost ini file
    with open(filename_ost,'w') as f:
        f.write(cal_ost_str)

    # Convert ini file to yaml file and remove old ini file
    subprocess.call(['rosrun', 'camera_calibration_parsers', 'convert', filename_ost, filename_yaml])
    os.remove(filename_ost)

    # Add autogenerated comment to top of file.
    with open(filename_yaml,'r') as f:
        yaml_lines = f.readlines()
    
    with open(filename_yaml,'w') as f:
        f.write('\n# Autogenerated configuration file - do not hand edit\n\n')
        for line in yaml_lines:
            f.write(line)
        f.write('\n\n')

def read_camera_calibration(camera_name):
    """
    Read camera calibration file and return calibration data.
    """
    if not os.path.isdir(camera_calibration_dir):
        os.mkdir(camera_calibration_dir)
    filename = os.path.join(config_dir,'cameras','calibrations', '{0}.yaml'.format(camera_name))
    with open(filename,'r') as f:
        calibration = yaml.load(f)
    return calibration
        
def read_target_info(name):
    """
    Reads the current target information form the target .yaml file.
    """
    filename = os.path.join(targets_dir,'{0}.yaml'.format(name))
    with open(filename,'r') as f: 
        target_info = yaml.load(f)
    target_info['square'] = str(target_info['square'])
    return target_info

def read_homography_calibrator_params():
    """
    Reads the homography calibrator parameters file
    """
    return read_yaml_file(homography_calibrator_params_file)

def write_homography_calibration(calibration_dict):
    """
    Write the homography calibration matrices to the homography
    directory.
    """
    for camera, matrix in calibration_dict.iteritems():
        filename = os.path.join(homographies_dir,'{0}.yaml'.format(camera))
        with open(filename,'w') as f:
            f.write('\n# Autogenerated configuration file - do not hand edit\n\n')
            yaml.dump({'rows' : matrix['rows']}, f, default_flow_style=False)
            yaml.dump({'cols' : matrix['cols']}, f, default_flow_style=False)
            yaml.dump({'data' : matrix['data']}, f, default_flow_style=False)

def read_homography_calibration(camera):
    """
    Reads the homography calibration for the given camera.
    """
    filename = os.path.join(homographies_dir,'{0}.yaml'.format(camera))
    with open(filename,'r') as f:
        homography_data = yaml.load(f)
    return homography_data

def write_transform_2d_calibration(calibration_dict):
    """
    Write the transform 2d calibrations to the transforms directory.
    """
    for camera_pair, transform in calibration_dict.iteritems():
        cam0, cam1 = camera_pair
        filename = os.path.join(transforms_2d_dir,'{0}_{1}.yaml'.format(cam0,cam1))
        with open(filename,'w') as f:
            f.write('\n# Autogenerated configuration file - do not hand edit\n\n')
            yaml.dump(transform, f, default_flow_style=False)

def read_transform_2d_calibration(camera0, camera1):
    """
    Reads the 2d transform calibration file for camera pairs camera0 and camera1
    """
    filename = os.path.join(transforms_2d_dir, '{0}_{1}.yaml'.format(camera0,camera1))
    return read_yaml_file(filename)

def get_transform_2d_calibration_files(fullpath=True):
    """
    Returns a list of the existing transform 2d calibration files.
    """
    file_list = os.listdir(transforms_2d_dir)
    dummy, params_file = os.path.split(transform_2d_calibrator_params_file)
    file_list.remove(params_file)
    if fullpath:
        file_list = [os.path.join(transforms_2d_dir,f) for f in file_list]
    return file_list

def get_last_modified_time(filename):
    """
    Returns the last modified time for the given file.
    """
    t_secs = os.path.getmtime(filename)
    t_struct = time.localtime(t_secs)
    t_string = time.strftime('%m/%d/%y-%H:%M:%S', t_struct) 
    return t_string 


def rsync_camera_calibrations(verbose=False):
    """
    Update the calibration files on the 
    """
    machine_def = read_machine_def()
    camera_assignment = read_camera_assignment()
    calibration_file_list = get_camera_calibration_files()
    user = machine_def['user']

    # Create dictionary relating cameras to existing calibration files
    calibration_file_dict = {}
    for camera in camera_assignment:
        camera_yaml = '{0}.yaml'.format(camera)
        for calibration_file in calibration_file_list:
            if camera_yaml in calibration_file:
                calibration_file_dict[camera] = calibration_file

    if verbose:
        print()

    for camera, calibration_file in calibration_file_dict.iteritems():

        # Get guid based name for camera calibration file and make temporary copy
        # in the calbirations directory
        camera_guid = camera_assignment[camera]['guid']
        guid_yaml = '{0}.yaml'.format(camera_guid)
        guid_yaml_temp = os.path.join(camera_calibration_dir,guid_yaml)
        shutil.copy(calibration_file, guid_yaml_temp)

        # Copy calibration file to camera computer using rsync
        computer = camera_assignment[camera]['computer']
        computer_address = machine_def[computer]['address']
        rsync_cmd_args = (guid_yaml_temp, user, computer_address, ros_camera_info_dir) 
        rsync_cmd = 'rsync -ae ssh {0} {1}@{2}:{3}/'.format(*rsync_cmd_args)
        if verbose:
            print('camera: {0}'.format(camera))
            print('address: {0}, mct computer: {1}'.format(computer_address, computer))
            print(rsync_cmd)
            print()
        rval = subprocess.call(rsync_cmd,shell=True)
        if rval != 0:
            raise IOError, 'error rsyncing calibration file'

        # Remove temporary guid calibration file
        os.remove(guid_yaml_temp)


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Development/testing

    if 0:
        params = read_mightex_params()
        print(params)
        for ctlr, ctlr_params in params.iteritems():
            print(ctlr)
            ctlr_params = ctlr_params.items()
            ctlr_params.sort()
            for chan, chan_params in ctlr_params:
                print('  ', chan, chan_params)

    if 0:
        params = {
                'ir_lighting': {
                    'channel_1': {'enabled': True, 'current': 500}, 
                    'channel_2': {'enabled': True, 'current': 500},
                    'channel_3': {'enabled': True, 'current': 500}, 
                    'channel_4': {'enabled': True, 'current': 500}, 
                    'port': '/dev/mightex-serial', 
                    }
                }
        write_mightex_params(params)


    if 0:
        frame_rates = read_frame_rates()
        print(frame_rates)
        
    if 0:
        machine_def = read_machine_def()
        print(machine_def)

    if 0:
        assignment = read_camera_assignment()
        for k,v in assignment.iteritems():
            print(k,v)

    if 0:
        target_info = read_target_info('chessboard')
        print('chessboard')
        print(target_info)
        print()
        target_info = read_target_info('active')
        print('active')
        print(target_info)
        print()

    if 0:
        t = get_last_modified_time('/home/albert/ros/mct_config/cameras/calibrations/camera_1.yaml')
        print(t)

    if 0:
        file_list = get_camera_calibration_files()
        for f in file_list:
            print(f)

    if 0:
        caldata = read_camera_calibration('camera_1')
        print(caldata)

    if 0:
        rsync_camera_calibrations(verbose=True)

    if 0:
        read_homography_calibrator_params()

    if 0:
        file_list = get_homography_calibration_files()
        print(file_list)
        file_list = get_homography_calibration_files()
        print(file_list)

    if 0:
        data = read_homography_calibration('camera_8')
        print(data)

    if 0:
        regions_dict = read_tracking_2d_regions()
        for region, camera_list in regions_dict.iteritems():
            print(region)
            for c in camera_list:
                print(' ', c)

    if 0:
        camera_pairs_dict = read_tracking_2d_camera_pairs()
        for region, pairs_list in camera_pairs_dict.iteritems():
            print(region)
            for pair in pairs_list:
                print(' ', pair)

    if 0:
        file_list = get_transform_2d_calibration_files()
        print(file_list)

    if 0:
        transform = read_transform_2d_calibration('camera_1', 'camera_4')
        print(transform)

    if 0:
        data = read_tracking_2d_stitching_params()
        print(data)

    if 0:
        params = {
                'brightness': 800, 
                'shutter': 250,
                'gain': 300,
                }

        camera_list = ['camera_{0}'.format(i) for i in range(1,12+1)]
        for camera in camera_list:
            write_camera_params(camera,params)

    if 0:
        camera_list = ['camera_{0}'.format(i) for i in range(1,12+1)]
        for camera in camera_list:
            params = read_camera_params(camera)
            print(camera, params)

    if 0:
        params = read_logging_params()
        print(params)

    if 1:
        params = read_logging_extra_video()
        print(params)


