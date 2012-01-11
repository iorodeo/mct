from __future__ import print_function
import os
import os.path
from xml.etree import ElementTree 


def read_machine_file():
    """
    Reads the machine definition xml ROS launch file  MCT_CONFIG/machine/mct.machine

    Returns a list containg a dictionary of attributes for each machine in the
    machine file.
    """
    mct_config_dir = os.environ['MCT_CONFIG']
    mct_machine_file = os.path.join(mct_config_dir, 'machine', 'mct.machine')
    tree = ElementTree.parse(mct_machine_file)
    machine_elem_list = tree.findall('machine')
    machine_list = [] 
    for machine_elem in machine_elem_list:
        machine_list.append(machine_elem.attrib)
    return machine_list



# -----------------------------------------------------------------------------
if __name__ == '__main__': 
    machine_list = read_machine_file()
    print(machine_list)


    

