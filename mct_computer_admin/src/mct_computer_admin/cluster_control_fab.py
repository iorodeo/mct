#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_computer_admin')

import sys
import os.path
import mct_xml_tools
import admin_tools
from subprocess import call
from fabric.api import *
from fabric.decorators import hosts
from fabric.contrib.files import exists

slave_list = admin_tools.get_slave_hosts()
host_list = admin_tools.get_hosts()
master = admin_tools.get_master()

msg_dict = {
        'wakeup'             : 'waking up camera computers',
        'push_setup'         : 'pushing setup to slave computers',
        'shutdown'           : 'shutting down camera computers',
        'list_slaves'        : 'listing slaves',
        'rospack_profile'    : 'run rospack profile on all slave computers', 
        'pull'               : 'pulling latest version of mct from repository on slave computers',
        'pull_master'        : 'pulling laster version of mct form repository on master',
        'pull_all'           : 'pull latest version of mct form repository for all computers in current machine',
        'clone'              : 'clone new version of mct from repository of slave computers',
        'clean'              : 'remove old version of mct',
        'rosmake'            : 'use rosmake to build mct on slave computers',
        'rosmake_preclean'   : 'use rosmake to build mct on slave computers',
        'update_machine_def' : 'updating the ROS xml machine launch file',
        'list_machine_def'   : 'listing current machine definition',
        'test'               : 'test command for development',
        }

def wakeup():
    """
    Wakes up camera computers using wake on lan. 
    """
    mac_list = admin_tools.get_slave_macs()
    for mac in mac_list:
        local('sudo etherwake -i eth1 {0}'.format(mac))

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
    slave_info = admin_tools.get_slave_info()
    for slave, info in slave_info.iteritems():
        print(slave,info)

def update_machine_def():
    """
    Updates the ROS xml machine launch file using the the current machine_def.yaml
    machine definition file.
    """
    machine_def = admin_tools.get_machine_def()
    mct_config = os.environ['MCT_CONFIG']
    mct_machine_file = os.path.join(mct_config,'machine','mct.machine')
    mct_xml_tools.launch.create_machine_launch(mct_machine_file,machine_def)

def list_machine_def():
    """
    Prints the current machine definition from the machine_def.yaml file.
    """
    machine_def = admin_tools.get_machine_def()
    print('user', machine_def['user'])
    print('master', machine_def['master'])

    slave_keys = machine_def.keys()
    slave_keys.remove('user')
    slave_keys.remove('master')
    slave_keys.sort()
    for key in slave_keys:
        print(key,machine_def[key])

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
        msg = msg_dict[cmd]
        print('\n{0}\n'.format(msg))
    except KeyError:
        print('Error: unknown command {0}'.format(cmd))
        sys.exit(0)

    # Generate fab command and add any arguments
    fab_cmd = 'fab -f {0} {1}'.format(fabfile, cmd,)
    if argv[2:]:
        fab_cmd_args = ','.join(argv[2:])
        fab_cmd = '{0}:{1}'.format(fab_cmd,fab_cmd_args)

    call(fab_cmd, shell=True)

