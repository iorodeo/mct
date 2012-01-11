from __future__ import print_function
import subprocess

def get_nodes():
    """
    Returns a list of all currently running nodes.
    """
    node_str = subprocess.check_output(['rosnode', 'list'])
    node_list = node_str.split()
    return node_list

def get_services():
    """
    Returns a list of all currently available services.
    """
    service_str = subprocess.check_output(['rosservice','list'])
    service_list = service_str.split()
    return service_list

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node_list = get_nodes()
    print(node_list)

    service_list = get_services()
    print(service_list)
