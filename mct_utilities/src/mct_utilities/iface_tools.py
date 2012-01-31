import netifaces

def get_ip_addr(iface): 
    """ 
    Returns the IP address for the given interface.  
    """ 
    ifaddresses = netifaces.ifaddresses(iface) 
    ip = ifaddresses[2][0]['addr'] 
    return ip

