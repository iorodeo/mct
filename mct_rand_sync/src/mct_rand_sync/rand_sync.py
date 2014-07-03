from __future__ import print_function
import roslib
roslib.load_manifest('mct_rand_sync')
import rospy

from mct_msg_and_srv.srv import GetRandSyncSignal 
from mct_msg_and_srv.srv import GetRandSyncTrigCnt
from mct_msg_and_srv.srv import ResetRandSync

def get_rand_sync_signal(seq):
    rospy.wait_for_service('/get_rand_sync_signal')
    try:
        proxy = rospy.ServiceProxy('/get_rand_sync_signal',GetRandSyncSignal)
        rsp = proxy(seq)
        status = rsp.status
        signal = rsp.signal
    except rospy.ServiceException, e:
        print('service call failed: {0}'.format(str(e)))
        status = False
        signal = None
    return status, signal

def get_rand_sync_trig_cnt():
    rospy.wait_for_service('/get_rand_sync_trig_cnt')
    try:
        proxy = rospy.ServiceProxy('/get_rand_sync_trig_cnt', GetRandSyncTrigCnt)
        rsp = proxy()
        status = rsp.status
        trig_cnt = rsp.trigCnt
    except rospy.ServiceException, e:
        print('service call failed: {0}'.format(str(e)))
        status = False
        trig_cnt = 0
    return status, trig_cnt

def reset_rand_sync():
    rospy.wait_for_service('/reset_rand_sync')
    rsp = None
    try:
        proxy = rospy.ServiceProxy('/reset_rand_sync', ResetRandSync)
        rsp = proxy()
        status = rsp.status
    except rospy.ServiceException, e:
        print('service call failed: {0}'.format(str(e)))
        status = False
    return status

     
# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import sys
    import time

    cmd = sys.argv[1]

    if cmd == 'cnt':
        print('getting trig cnt')
        status, cnt = get_rand_sync_trig_cnt()
        print('cnt = {0}'.format(cnt))

    elif cmd == 'reset':
        print('resetting trig cnt')
        reset_rand_sync()

    elif cmd == 'sync':
        print('getting sync')
        for i in range(100):
            status, cnt = get_rand_sync_trig_cnt()
            if status == False:
                print('failed to get cnt')
            else:
                status, signal = get_rand_sync_signal(cnt)
                print('cnt = {0}, signal = {1}'.format(cnt,signal))
            time.sleep(0.1)

    elif cmd == 'sync-repeat':
        print('running sync-repeat (Ctl-C to exit)')
        while True:
            status, cnt = get_rand_sync_trig_cnt()
            status, signal = get_rand_sync_signal(cnt)
            print('cnt = {0}, signal = {1}'.format(cnt,signal))
            time.sleep(0.1)

    else:
        print('\nuknown command')
        




    
