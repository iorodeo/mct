from __future__ import print_function
import os
import os.path
import yaml

def get_machine_def():
    """
    Reads the machine definition file.
    """
    config_pkg = os.environ['MCT_CONFIG']
    machine_def_file = os.path.join(config_pkg,'machine','machine_def.yaml')
    with open(machine_def_file,'r') as f:
        machine_def = yaml.load(f)
    return machine_def

def get_slave_info():
    """
    Reads the machine definition file and gets the slave information.
    """
    slave_info = get_machine_def()
    slave_info.pop('mct_master') 
    slave_info.pop('user')
    return slave_info

def get_hosts():
    """
    Returns a list of all hosts in the current machine
    """
    machine_def = get_machine_def()
    hosts = []
    for k,v in machine_def.iteritems():
        try:
            hosts.append(v['address'])
        except KeyError:
            pass
        except TypeError:
            pass
    return hosts

def get_slave_hosts():
    """
    Returns a list of the slave host names
    """
    slave_info = get_slave_info()
    return [v['address'] for k,v in slave_info.iteritems()]

def get_slave_macs():
    """
    Return a list of the slave mac addresses
    """
    slave_info = get_slave_info()
    return [v['mac'] for k,v in slave_info.iteritems()]

def get_master():
    """
    Get the host name of the master computer.
    """
    machine_def = get_machine_def()
    return machine_def['mct_master']['address']



# -----------------------------------------------------------------------------
if __name__ == '__main__':

    machine_def = get_machine_def()
    print(machine_def)

    slave_info = get_slave_info()
    print(slave_info)

    slave_hosts = get_slave_hosts()
    print(slave_hosts)

    slave_macs = get_slave_macs()
    print(slave_macs)

    hosts = get_hosts()
    print(hosts)
            

