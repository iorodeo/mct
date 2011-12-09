#!/usr/bin/env python
from __future__ import print_function
import sys
import os.path
from subprocess import call
from fabric.api import local
from fabric.api import sudo 
from fabric.decorators import hosts
from admin_tools import get_slave_info

slave_info = get_slave_info()

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


def main(argv):
    # Get command line argument
    cmd = argv[1].lower()
    
    if cmd == 'wakeup':
        print('waking up camera computers')
    elif cmd == 'shutdown':
        print('shutting down camera computers')
    else:
        print('Error: unknown command {0}'.format(cmd))

    fabfile, ext = os.path.splitext(__file__)
    fabfile = '{0}.py'.format(fabfile)
    call('fab -f {0} {1}'.format(fabfile, cmd), shell=True)

