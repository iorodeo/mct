from __future__ import print_function
import roslib
roslib.load_manifest('mct_logging')
import rospy

from mct_msg_and_srv.srv import LoggingCmd

def tracking_pts_logger_proxy(namespace,cmd,filename):
    """
    Proxy function for the tracking pts logger service. Used to start and stop 
    logging.
    """
    srv = '{0}/logging_cmd'.format(namespace)
    proxy = rospy.ServiceProxy(srv,LoggingCmd)
    try:
        resp = proxy(cmd,filename)
        flag = resp.flag
    except rospy.ServiceException, e:
        flag = False
    return flag

def start_logging(namespace,filename):
    return tracking_pts_logger_proxy(namespace,'start',filename)

def stop_logging(namespace):
    return tracking_pts_logger_proxy(namespace,'stop','')

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import sys
    import os
    import os.path

    region = sys.argv[1]
    cmd = sys.argv[2].lower()

    namespace = '{0}/tracking_pts_logger'.format(region)
    filename = os.path.join(os.environ['HOME'], '{0}_tracking_pts.json'.format(region))

    if cmd == 'start':
        start_logging(namespace,filename)
    elif cmd == 'stop':
        stop_logging(namespace)
    else:
        print('unknown command {0}'.format(cmd))

