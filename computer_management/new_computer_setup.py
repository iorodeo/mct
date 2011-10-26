from fabric.api import *
import os

env.user = 'albert'
env.path = '/home/albert'
env.hosts = []

def setup_ssh_keys():
    global env
    if len(env.hosts) == 0:
        host_string = prompt('Enter hostname(s):')
        env.hosts = host_string.split(',')
    ssh_directory = '~/.ssh/'
    print 'Public ssh keys found in ' + ssh_directory + ' :'
    ssh_directory = os.path.expanduser(ssh_directory)
    file_list = os.listdir(ssh_directory)
    key_list = [os.path.splitext(file)[0] for file in file_list if os.path.splitext(file)[1] == '.pub']
    print key_list
    key_string = prompt('Enter the name(s) of the ssh key you wish to copy to hosts:')
    keys = key_string.split(',')
    keys = [ssh_directory + key for key in keys]
    for host in env.hosts:
        for key in keys:
            local('ssh-copy-id -i ' + key + ' ' + env.user + '@' + host)

def setup_ros():
    for host in env.hosts:
        with settings(host_string=host,
                      warn_only=True):
            with cd('~'):
                run('mkdir ros')
                with cd('ros'):
                    run('hg clone http://bitbucket.org/iorodeo/multi_cam_tracker')
                run('echo "source /opt/ros/diamondback/setup.bash" >> ~/.bashrc')
                run('echo "export ROS_PACKAGE_PATH=~/ros:$ROS_PACKAGE_PATH" >> ~/.bashrc')
                run('echo "export ROS_MASTER_URI=http://felis:11311" >> ~/.bashrc')
                run('. ~/.bashrc')


def setup_udev():
    for host in env.hosts:
        with settings(host_string=host,
                      warn_only=True):
            put('99-camera1394.rules','/etc/udev/rules.d/',use_sudo=True,mirror_local_mode=True)
            sudo('addgroup video')
            sudo('adduser $USER video')
            sudo('reboot')

def clean():
    if len(env.hosts) == 0:
        host_string = prompt('Enter hostname(s):')
        env.hosts = host_string.split(',')
    for host in env.hosts:
        with settings(host_string=host,
                      warn_only=True):
            print 'removing authorized_keys for host ' + host
            run('rm ~/.ssh/authorized_keys')
            print 'removing 99-camera1394.rules for host ' + host
            sudo('rm /etc/udev/rules.d/99-camera1394.rules')
            print 'removing ~/ros/'
            run('rm -rf ~/ros/')

if __name__ == "__main__":
    setup_ssh_keys()
    setup_ros()
    setup_udev()
    # clean()
