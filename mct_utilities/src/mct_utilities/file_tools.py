from __future__ import print_function
import os
import os.path
import yaml
import subprocess
import time

config_dir = os.environ['MCT_CONFIG']
calibration_dir = os.path.join(config_dir,'cameras', 'calibrations')

def write_camera_assignment(yaml_dict):
    """
    Write camera assignment to mct configuration directory
    """
    # Write yaml file to 'camera_assignment.yaml' in cameras section of mct configuration
    filename = os.path.join(config_dir, 'cameras', 'camera_assignment.yaml')
    with open(filename,'w') as f:
        f.write('\n# Autogenerated configuration file - do not hand edit\n\n')
        camera_name_list = yaml_dict.keys()
        camera_name_list.sort
        yaml.dump(yaml_dict,f,default_flow_style=False)

def read_camera_assignment():
    """
    Reads the current camera assignment from the camera assignment yaml file. If 
    this file doesn't exist then None is returned.
    """
    filename = os.path.join(config_dir, 'cameras', 'camera_assignment.yaml')
    if os.path.isfile(filename):
        try:
            with open(filename) as f:
                yaml_dict = yaml.load(f)
        except:
            return None
        return yaml_dict
    else:
        return None

def get_camera_calibration_files(fullpath=True):
    """
    Returns a list of the camera calibration yaml files in the calibration directory of
    the mct configuration.
    """
    file_list = os.listdir(calibration_dir)
    file_list = [f for f in file_list if 'camera_' in f]
    if fullpath:
        file_list = [os.path.join(calibration_dir,f) for f in file_list]
    return file_list

def write_camera_calibration(camera_name, cal_ost_str):
    """
    Writes the camera calibration given the camera name and calibration ost
    string produced by the cameracalibrator node.

    This process is a bit convoluted - but I want to stick with the standard
    ROS tools while still adding the "Autogenerated" tag to the top of the yaml
    files.
    """
    if not os.path.isdir(calibration_dir):
        os.mkdir(calibration_dir)
    basename = os.path.join(calibration_dir, camera_name)
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
    if not os.path.isdir(calibration_dir):
        os.mkdir(calibration_dir)
    filename = os.path.join(config_dir,'cameras','calibrations', '{0}.yaml'.format(camera_name))
    with open(filename,'r') as f:
        calibration = yaml.load(f)
    return calibration
        
def read_target_info(name):
    """
    Reads the current target information form the target .yaml file.
    """
    targets_dir = os.path.join(config_dir, 'targets')
    filename = os.path.join(targets_dir,'{0}.yaml'.format(name))
    with open(filename,'r') as f: 
        target_info = yaml.load(f)
    target_info['square'] = str(target_info['square'])
    return target_info

def get_last_modified_time(filename):
    """
    Returns the last modified time for the given file.
    """
    t_secs = os.path.getmtime(filename)
    t_struct = time.localtime(t_secs)
    t_string = time.strftime('%m/%d/%y-%H:%M:%S', t_struct) 
    return t_string 

# -----------------------------------------------------------------------------
if __name__ == '__main__':

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

    if 1:
        caldata = read_camera_calibration('camera_1')
        print(caldata)

