#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_computer_admin')
import rospy

import sys
import time
import os.path
import tempfile
import subprocess
import mct_xml_tools
import mct_introspection
import mct_utilities

from fabric.api import *
from fabric.decorators import hosts
from fabric.contrib.files import exists

slave_list = mct_introspection.get_slave_hosts()
host_list = mct_introspection.get_hosts()
master = mct_introspection.get_master()

cmd_msgs = {
        'wakeup': 'waking up camera computers',
        'push_setup': 'pushing setup to slave computers',
        'shutdown': 'shutting down camera computers',
        'list_slaves': 'listing slaves',
        'rospack_profile': 'run rospack profile on all slave computers', 
        'pull': 'pulling latest version of the mct from repository to slave computers',
        'pull_master': 'pulling laster version of the mct form repository on master',
        'pull_all': 'pull latest version of the mct repository for all computers in current machine',
        'pull_from_master': 'pull latest version of the mct repository from the master to the slaves', 
        'clone': 'clone new version of mct from repository of slave computers',
        'clean': 'remove old version of mct',
        'rosmake': 'use rosmake to build mct on slave computers',
        'rosmake_preclean': 'use rosmake to build mct on slave computers',
        'update_machine_def': 'updating the ROS xml machine launch file',
        'list_machine_def': 'listing current machine definition',
        'list_cameras': 'listing cameras',
        'list_camera_assignment': 'listing camera assignment', 
        'rsync_camera_calibrations': 'rsyncing camera calibraitons',
        'clean_camera_calibrations': 'cleaning camera calibrations',
        'show_camera_info':  'launching camera_info viewers',
        'show_camera_info_header': 'launch camera_info/header viewers',
        'show_camera_info_header_seq': 'launch camera_info/header/seq viewers',
        'test': 'test command for development',
        'camera_assignment': 'starting camera assigment application',
        'zoom_calibration': 'starting zoom calibration application',
        'camera_calibration': 'starting the camera calibration application',
        'homography_calibration': 'starting the homography calibration application',
        'transform_2d_calibration': 'starting the 2d transform calibration application',
        'tracking_2d': 'starting the 2d tracking application',
        'help': 'Command line interface to MCT (Multi-Camera Tracker)',
        'frame_drop_test':  '2d tracking application w/ frame dropped frame logger',
        }

fab_cmds = [ 
        'wakeup', 
        'push_setup', 
        'shutdown', 
        'list_slaves', 
        'rospack_profile', 
        'pull', 
        'pull_master',
        'pull_all',
        'pull_from_master',
        'clone',
        'clean',
        'rosmake',
        'rosmake_preclean',
        'clean_camera_calibrations',
        ]

def help():
    print('Usage: mct [command]')
    print()
    print('Commands:')
    print()
    print('  Basic Administration')
    print('  --------------------')
    print('    wakeup               - wake up slave computers')
    print('    shutdown             - shutdown slave computers ')
    print('    update_machine_def   - updates the machine definition file')
    print()
    print('  Calibration (in order of application)')
    print('  -------------------------------------')
    print('    camera_assignment          - run the camera assigment application')
    print('    zoom_calibration           - run the camera zoom calibration application')
    print('    camera_calibration         - run the camera calibration applicaiton')
    print('    homography_calibration     - run the homography calibration application')
    print('    transform_2d_calibration   - run the 2d transform calibration application')
    print()
    print('  Tracking')
    print('  --------')
    print('    tracking_2d   - 2D tracking applicaton')
    print()
    print('  Testing')
    print('  -------')
    print('    frame_drop_test - 2D tracking application w/ dropped frame logger')
    print()

def camera_assignment():
    """
    Starts the camera assignment application.
    """
    roslaunch('camera_assignment.launch')

def zoom_calibration():
    """
    Starts the zoom calibration tool.
    """
    roslaunch('zoom_calibration.launch')

def camera_calibration():
    """
    Starts the camera calibration application
    """
    roslaunch('camera_calibration.launch')

def homography_calibration():
    """
    Starts the homography calibration application
    """
    roslaunch('homography_calibration.launch')

def transform_2d_calibration():
    """
    Starts the 2d transformation calibration application
    """
    roslaunch('transform_2d_calibration.launch')

def tracking_2d():
    """
    Starts the tracking 2d application
    """
    roslaunch('tracking_2d.launch')

def frame_drop_test():
    """
    Starts the frame drop test application
    """
    roslaunch('frame_drop_test.launch')

def roslaunch(launch_file):
    """
    Runs a roslaunch file.
    """
    try:
        subprocess.call(['roslaunch', 'mct_launch', launch_file])
    except KeyboardInterrupt:
        return

def wakeup():
    """
    Wakes up camera computers using wake on lan. 
    """
    mac_and_iface_list = mct_introspection.get_slave_mac_and_iface()
    for mac, iface in mac_and_iface_list:
        local('sudo etherwake -i {0} {1}'.format(iface,mac))

@hosts(*slave_list)
def shutdown(): 
    """
    Shutdown camera computers
    """
    sudo('halt',shell=False)

@hosts(*slave_list)
def push_setup():
    """
    Pushes mct_setup.bash file to the slave computers
    """
    bin_dir = os.path.join(os.environ['HOME'],'bin')
    setup_file = os.path.join(bin_dir,'mct_setup.bash')
    if not exists(bin_dir):
        run('mkdir {0}'.format(bin_dir))
    with cd(bin_dir):
        put(setup_file,setup_file)

def _pull():
    """
    Pull lastest version of mct from the repository.
    """
    mct_name = os.environ['MCT_NAME']
    if exists(mct_name):
        with cd(mct_name):
            run('hg pull -u')
    else:
        print('ERROR: unable to pull and update {0} does not exits'.format(mct_name))

@hosts(*slave_list)
def pull():
    """
    Pull lastest version of mct from the repository on slave computers only.
    """
    _pull()

@hosts(master)
def pull_master():
    """
    Pull latest version of mct from repository on master computer only.
    """
    _pull()

@hosts(*host_list)
def pull_all():
    """
    Pull latest version for mct repository on all computers in machine
    """
    _pull()

@hosts(*slave_list)
def pull_from_master():
    """
    Pull the latest version of the mct repository from the master computer.
    """
    mct_name = os.environ['MCT_NAME']
    mct_name_split = mct_name.split('/')
    mct_local = '/'.join(mct_name_split[-2:])
    machine_def = mct_introspection.get_machine_def()
    user = machine_def['user']
    address = machine_def['mct_master']['address']
    cmd = 'hg pull -u ssh://{0}@{1}/{2}'.format(user,address,mct_local)
    if exists(mct_name):
        with cd(mct_name):
            run(cmd)

def _rosmake(preclean=False):
    """
    Run rosmake on mct project directory
    """
    mct_name = os.environ['MCT_NAME']
    if exists(mct_name):
        with cd(mct_name):
            if preclean:
                run('source ~/bin/mct_setup.bash; rosmake --rosdep-install --pre-clean',shell=True)
            else:
                run('source ~/bin/mct_setup.bash; rosmake --rosdep-install',shell=True)

    else:
        print('ERROR: unable to run rosamke {0} does not exist'.format(mct_name))


@hosts(*slave_list)
def rosmake():
    """
    Run rosmake on mct project directory on remote computers
    """
    _rosmake(preclean=False)

@hosts(*slave_list)
def rosmake_preclean():
    """
    Run rosmake on mct project directory on remote computers - preclean before makeing.
    """
    _rosmake(preclean=True)

@hosts(*slave_list)
def clone():
    """
    Clone a new versoin of mct on the slave computers.

    Note, may want to move bitbucket address to configuration file.
    """
    ros_local = os.environ['ROS_LOCAL']
    mct_name = os.environ['MCT_NAME']
    if not exists(ros_local):
        run('mkdir {0}'.format(ros_local))
    with cd(ros_local):
        if exists(mct_name):
            run('rm -r {0}'.format(mct_name))
        run('hg clone http://bitbucket.org/iorodeo/mct')

@hosts(*slave_list)
def clean():
    """
    Remove any old versions of mct, the contents of the .mct directory, and the
    bash setup file mct_setup.bash.
    """
    mct_name = os.environ['MCT_NAME']
    if exists(mct_name):
        run('rm -r {0}'.format(mct_name))
    mct_resources = os.environ['MCT_RESOURCES'] 
    if exists(mct_resources):
        run('rm -r {0}'.format(mct_resources))
    bin_dir = os.path.join(os.environ['HOME'],'bin')
    setup_file = os.path.join(bin_dir,'mct_setup.bash')
    run('rm {0}'.format(setup_file))

@hosts(*slave_list)
def rospack_profile():
    """
    Run rospack profile on all slave computers.
    """
    run('source ~/bin/mct_setup.bash; rospack profile',shell=True)

def list_slaves():
    """
    List slaves names and mac addresses
    """
    slave_info = mct_introspection.get_slave_info()
    for slave, info in slave_info.iteritems():
        print(slave,info)

def update_machine_def():
    """
    Updates the ROS xml machine launch file using the the current machine_def.yaml
    machine definition file.
    """
    machine_def = mct_introspection.get_machine_def()
    machine_dir = mct_utilities.file_tools.machine_dir
    mct_machine_file = os.path.join(machine_dir, 'mct.machine')
    mct_xml_tools.launch.create_machine_launch(mct_machine_file, machine_def)

def list_machine_def():
    """
    Prints the current machine definition from the machine_def.yaml file.
    """
    machine_def = mct_introspection.get_machine_def()
    print('user', machine_def['user'])
    print('master', machine_def['mct_master'])

    slave_keys = machine_def.keys()
    slave_keys.remove('user')
    slave_keys.remove('mct_master')
    slave_keys.sort()
    for key in slave_keys:
        print(key,machine_def[key])

def list_cameras():
    """
    Lists all comands currently connected to the system.
    """
    inspector_popen = subprocess.Popen(['roslaunch', 'mct_camera_tools', 'camera1394_inspector_master.launch'])
    time.sleep(3.0) # Wait fo processes to start - kludgey need a better approach
    camera_dict = mct_introspection.find_cameras()
    print('\nfound {0} cameras\n'.format(len(camera_dict)))
    for k,v in camera_dict.iteritems():
        print('guid {0}, {1}'.format(k,v))
    print('\n')
    inspector_popen.send_signal(subprocess.signal.SIGINT) 

def list_camera_assignment():
    """
    List the current camera assignment as given in the camera_assignment.yaml file.
    """
    camera_assignment = mct_introspection.get_camera_assignment()
    if camera_assignment is not None:
        camera_nums = camera_assignment.keys()

        # Sort cameras in numerical order
        def cmp_func(x,y):
            num_x = int(x.replace('camera_', ''))
            num_y = int(y.replace('camera_', ''))
            if num_x > num_y:
                return 1
            elif num_x < num_y:
                return -1
            else:
                return 0
        camera_nums.sort(cmp=cmp_func) 
        # Display camera assignment
        for cam in camera_nums: 
            print(cam)
            for k,v in camera_assignment[cam].iteritems():
                print('  {0}: {1}'.format(k,v))
        print()
    else:
        print('\n camera assignment is None\n')

def rsync_camera_calibrations():
    """
    Use rsync to send camera calibrations to camera computers
    """
    mct_utilities.file_tools.rsync_camera_calibrations(verbose=True)

@hosts(*host_list)
def clean_camera_calibrations():
    """
    Clean any existing old camera calibration files.
    """
    ros_camera_info_dir = mct_utilities.file_tools.ros_camera_info_dir
    if exists(ros_camera_info_dir):
        with cd(ros_camera_info_dir):
            run('rm *.yaml')

def show_camera_info():
    """
    Launch rostopic echo for all camera info topics.
    """
    camera_info_topics = mct_introspection.find_camera_info_topics()
    popen_list = []
    filename_list = []
    tmp_dir = tempfile.gettempdir()
    pos_x, pos_y, step_x, step_y = 10, 50, 170, 0
    for i, topic in enumerate(camera_info_topics):
        filename = os.path.join(tmp_dir, 'camera_info_{0}.bash'.format(i))
        with open(filename, 'w') as f:
            f.write('rostopic echo {0}\n'.format(topic))
        os.chmod(filename,0755)
        geometry = '{0}x{1}+{2}+{3}'.format(80,28,pos_x,pos_y)
        popen = subprocess.Popen(['gnome-terminal', '--geometry', geometry, '-x', filename])
        popen_list.append(popen)
        filename_list.append(filename)
        pos_x += step_x
        pos_y += step_y
        time.sleep(0.1)
    # Wait a bit before deleting files
    time.sleep(5)
    for filename in filename_list:
        os.remove(filename)

def show_camera_info_header():
    """
    Launch rostopic echo for all camera_info/header topics. Note, This function
    should be unified with show_camera_info above.
    """
    camera_info_topics = mct_introspection.find_camera_info_topics()
    popen_list = []
    filename_list = []
    tmp_dir = tempfile.gettempdir()
    pos_x, pos_y, step_x, step_y = 10, 50, 170, 0
    for i, topic in enumerate(camera_info_topics):
        filename = os.path.join(tmp_dir, 'camera_info_{0}.bash'.format(i))
        with open(filename, 'w') as f:
            f.write('rostopic echo {0}/header\n'.format(topic))
        os.chmod(filename,0755)
        geometry = '{0}x{1}+{2}+{3}'.format(80,28,pos_x,pos_y)
        popen = subprocess.Popen(['gnome-terminal', '--geometry', geometry, '-x', filename])
        popen_list.append(popen)
        filename_list.append(filename)
        pos_x += step_x
        pos_y += step_y
        time.sleep(0.1)
    # Wait a bit before deleting files
    time.sleep(5)
    for filename in filename_list:
        os.remove(filename)

def show_camera_info_header_seq():
    """
    Launch rostopic echo for all camera_info/header topics. Note, This function
    should be unified with show_camera_info above.
    """
    camera_info_topics = mct_introspection.find_camera_info_topics()
    popen_list = []
    filename_list = []
    tmp_dir = tempfile.gettempdir()
    pos_x, pos_y, step_x, step_y = 10, 50, 170, 0
    for i, topic in enumerate(camera_info_topics):
        filename = os.path.join(tmp_dir, 'camera_info_{0}.bash'.format(i))
        with open(filename, 'w') as f:
            f.write('rostopic echo {0}/header/seq\n'.format(topic))
        os.chmod(filename,0755)
        geometry = '{0}x{1}+{2}+{3}'.format(80,28,pos_x,pos_y)
        popen = subprocess.Popen(['gnome-terminal', '--geometry', geometry, '-x', filename])
        popen_list.append(popen)
        filename_list.append(filename)
        pos_x += step_x
        pos_y += step_y
        time.sleep(0.1)
    # Wait a bit before deleting files
    time.sleep(5)
    for filename in filename_list:
        os.remove(filename)
        

def test(*args):
    print('test')

def main(argv):
    """
    Main function for mct_cluster_control when called as a commandline program.
    """
    # Get command line argument
    cmd = argv[1].lower()
    fabfile, ext = os.path.splitext(__file__)
    fabfile = '{0}.py'.format(fabfile)
    try: 
        msg = cmd_msgs[cmd]
        print('\n{0}\n'.format(msg))
    except KeyError:
        print('Error: unknown command {0}'.format(cmd))
        sys.exit(0)

    # Generate fab command and add any arguments
    if cmd in fab_cmds:
        fab_cmd = 'fab -f {0} {1}'.format(fabfile, cmd,)
        if argv[2:]:
            fab_cmd_args = ','.join(argv[2:])
            fab_cmd = '{0}:{1}'.format(fab_cmd,fab_cmd_args)

        subprocess.call(fab_cmd, shell=True)
    else:
        try:
            globals()[cmd]() 
        except KeyError:
            print('ERROR: command not found, {0}'.format(cmd))

