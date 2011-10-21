from fabric.api import *
import os.path

env.user = 'albert'
env.path = '/home/albert'
env.key_filename = ['~/.ssh/test_key','~/.ssh/jabberwocky_peter_rsa']

def setup():
    host_string = prompt('Enter hostname(s):')
    hosts = host_string.split(',')
    # print hosts
    # ssh_key_string = prompt('Enter path to ssh key(s):')
    # ssh_keys = ssh_key_string.split(',')
    # print ssh_keys

    for host in hosts:
        # with settings(abort_on_prompts=True):
        #     print env
        #     local('ssh ' + env.user + '@' + host)
        for key in env.key_filename:
            key = os.path.expanduser(key)
            with settings(warn_only=True):
                output = local("test -f " + key,capture=True)
                if output.succeeded:
                    for host in hosts:
                        local('ssh-copy-id -i ' + key + ' ' + env.user + '@' + host)
                        put('99-camera1394.rules','/etc/udev/rules.d/',use_sudo=True,mirror_local_mode=True)

    # print env
    # print "host = " + host
    # key = os.path.expanduser(key)
    # with settings(host=host,
    #               host_string=host,
    #               hosts=[host],
    #               all_hosts=[host],
    #               key_filename = [key]):
    #     local('ssh-copy-id -i ' + key + ' ' + env.user + '@' + host)
    #     with cd('~'):
    #         run('mkdir ros')
