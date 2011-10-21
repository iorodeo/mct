from fabric.api import *
import os.path

env.user = 'albert'
env.hosts = ['felis', 'tabby']
env.path = '/home/albert'
# env.key_filename = ['/home/wbd/.ssh/test_key','/home/peter/.ssh/jabberwocky_peter_rsa']


def clone():
    # run('mkdir ros')
    with cd('ros'):
        run('hg clone http://bitbucket.org/iorodeo/multi_cam_tracker')

def update():
    with cd('ros/multi_cam_tracker'):
        run('hg pull -u')

