from fabric.api import *

env.user = 'albert'
env.hosts = ['felis', 'tabby']
env.path = '/home/albert'
env.key_filename = '/home/wbd/.ssh/test_key'


def test():
    run('mkdir fab_test_dir')
    with cd('fab_test_dir'):
        run('hg clone http://bitbucket.org/iorodeo/py2scad')

def clean_test():
    run('rm -r fab_test_dir')

