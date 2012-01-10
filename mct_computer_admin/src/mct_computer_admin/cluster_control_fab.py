#!/usr/bin/env python
from __future__ import print_function
import sys
import os.path
from subprocess import call
from fabric.api import *
from fabric.decorators import hosts
from fabric.contrib.files import exists
from admin_tools import get_slave_info

slave_info = get_slave_info()

msg_dict = {
        'wakeup'           : 'waking up camera computers',
        'push_setup'       : 'pushing setup to slave computers',
        'shutdown'         : 'shutting down camera computers',
        'list_slaves'      : 'listing slaves',
        'pull'             : 'pulling latest version of mct from repository on slave computers',
        'clone'            : 'clone new version of mct from repository of slave computers',
        'clean'            : 'remove old version of mct',
        'rosmake'          : 'use rosmake to build mct on slave computers',
        'rosmake_preclean' : 'use rosmake to build mct on slave computers',
        }

def wakeup():
    """
    Wakes up camera computers using wake on lan. 
    """
    for name, mac in slave_info.iteritems():
        local('sudo etherwake -i eth1 {0}'.format(mac))

@hosts(*slave_info.keys())
def shutdown(): 
    """
    Shutdown camera computers
    """
    sudo('halt',shell=False)

@hosts(*slave_info.keys())
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

@hosts(*slave_info.keys())
def pull():
    """
    Pull latest version of mct repositoy on slave computers.
    """
    mct_name = os.environ['MCT_NAME']
    if exists(mct_name):
        with cd(mct_name):
            run('hg pull -u')
    else:
        print('ERROR: unable to pull and update {0} does not exits'.format(mct_name))

@hosts(*slave_info.keys())
def rosmake():
    """
    Run rosmake on mct project directory on remote computers
    """
    mct_name = os.environ['MCT_NAME']
    if exists(mct_name):
        with cd(mct_name):
            run('source ~/bin/mct_setup.bash; rosmake --rosdep-install',shell=True)
    else:
        print('ERROR: unable to run rosamke {0} does not exist'.format(mct_name))

@hosts(*slave_info.keys())
def rosmake_preclean():
    """
    Run rosmake on mct project directory on remote computers - preclean before makeing.
    """
    mct_name = os.environ['MCT_NAME']
    if exists(mct_name):
        with cd(mct_name):
            run('source ~/bin/mct_setup.bash; rosmake --rosdep-install --pre-clean',shell=True)
    else:
        print('ERROR: unable to run rosamke {0} does not exist'.format(mct_name))

@hosts(*slave_info.keys())
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

@hosts(*slave_info.keys())
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

def list_slaves():
    """
    List slaves names and mac addresses
    """
    for name, mac in slave_info.iteritems():
        print('{0} {1}'.format(name,mac))

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

    call('fab -f {0} {1}'.format(fabfile, cmd), shell=True)

