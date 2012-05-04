from __future__ import print_function
import roslib
roslib.load_manifest('mct_logging')
import rospy

from mct_msg_and_srv.srv import CommandString 

def tracking_pts_logger_master_proxy(cmd):
    srv = '/tracking_pts_logger_master'
    proxy = rospy.ServiceProxy(srv,CommandString)
    try:
        resp = proxy(cmd)
        flag = resp.flag
        message = resp.message
    except rospy.ServiceException, e:
        flag = False
        message = str(e)
    return flag, message

def start_tracking_pts_loggers():
    tracking_pts_logger_master_proxy('start')

def stop_tracking_pts_loggers():
    tracking_pts_logger_master_proxy('stop')

# -----------------------------------------------------------------------
if __name__ == '__main__':
    """ Testing  """
    import sys
    cmd = sys.argv[1]
    if cmd == 'start':
        start_tracking_pts_loggers()
    elif cmd == 'stop':
        stop_tracking_pts_loggers()
    else:
        print('Error: unknown command {0}'.format(cmd))

