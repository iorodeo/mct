import os
import os.path

def get_slave_info():
    """
    Reads the slave_macs file in the configuration machine/slave_macs 
    file. This file contiains a list of hostnames and mac addresses.
    """
    config_pkg = os.environ['MCT_CONFIG']
    machine_dir = os.path.join(config_pkg,'machine')
    slave_macs_file = os.path.join(machine_dir,'slave_macs')

    print slave_macs_file

    with open(slave_macs_file,'r') as f:
        slave_dict = {}
        for line in f.readlines():
            line = line.split()
            hostname = line[0]
            macaddress = line[1]
            slave_dict[hostname] = macaddress

    return slave_dict
            

