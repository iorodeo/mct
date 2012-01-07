#!/usr/bin/env python
from __future__ import print_function
import sys
import os.path
from subprocess import call
from fabric.api import local
from fabric.api import sudo 
from fabric.api import env
from fabric.decorators import hosts
from admin_tools import get_slave_info

slave_info = get_slave_info()

msg_dict = {
        'wakeup'         : 'waking up camera computers',
        'shutdown'       : 'shutting down camera computers',
        'push_mct_setup' : 'pushing mct_setup.bash to camera computers',
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
    Shutsdown camera computers
    """
    sudo('halt',shell=False)

@hosts(*slave_info.keys())
def push_mct_setup():
    """
    Pushes the mct_setup.bash file to the camera computers.
    """
    for k,v in env.iteritems():
        print('{0} {1}'.format(k,v))

def main(argv):
    # Get command line argument
    cmd = argv[1].lower()

    fabfile, ext = os.path.splitext(__file__)
    fabfile = '{0}.py'.format(fabfile)

    try: 
        msg = msg_dict[cmd]
    except KeyError:
        print('Error: unknown command {0}'.format(cmd))
        sys.exit(0)

    call('fab -f {0} {1}'.format(fabfile, cmd), shell=True)

