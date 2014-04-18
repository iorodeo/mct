#!/usr/bin/env python
import roslib
roslib.load_manifest('mct_rand_sync')
import threading
import rospy
from mct_rand_sync import RandSyncDev

from mct_msg_and_srv.srv import GetRandSyncSignal 
from mct_msg_and_srv.srv import GetRandSyncSignalResponse
from mct_msg_and_srv.srv import GetRandSyncTrigCnt 
from mct_msg_and_srv.srv import GetRandSyncTrigCntResponse
from mct_msg_and_srv.srv import ResetRandSync
from mct_msg_and_srv.srv import ResetRandSyncResponse 
from mct_msg_and_srv.srv import CommandString
from mct_msg_and_srv.srv import CommandStringResponse


class RandSyncNode(object):

    def __init__(self):

        self.dev = None

        # NOTE: add port and buadrate to configuration system
        self.dev = RandSyncDev(port='/dev/ttyUSB0',baudrate=115200)

        rospy.init_node("rand_sync")
        rospy.on_shutdown(self.shutdown)

        # Setup services
        self.sync_signal_srv = rospy.Service(
                'get_rand_sync_signal',
                GetRandSyncSignal,
                self.get_rand_sync_signal
                )
        self.trig_cnt_srv = rospy.Service(
                'get_rand_sync_trig_cnt',
                GetRandSyncTrigCnt,
                self.get_rand_sync_trig_cnt
                )
        self.reset_srv = rospy.Service(
                'reset_rand_sync',
                ResetRandSync,
                self.reset_rand_sync
                )

    def get_rand_sync_signal(self,req):
        status = True
        message = ''
        sync_signal = (0,0,0)
        try:
            sync_signal = self.dev.getSyncSignal(req.seq)
        except (ValueError, IOError), e:
            status = False
            message = str(e)
        return GetRandSyncSignalResponse(status,message,sync_signal)

    def get_rand_sync_trig_cnt(self,req):
        status = True
        message = ''
        trigCnt = 0
        try:
            trigCnt = self.dev.getTrigCnt()
        except (ValueError, IOError), e:
            status = False
            message = str(e)
        return GetRandSyncTrigCntResponse(status, message, trigCnt)

    def reset_rand_sync(self,req):
        print 'reset'
        status = True
        message = ''
        self.dev.resetTrigCnt()
        return ResetRandSyncResponse(status, message)

    def run(self):
        rospy.spin()

    def shutdown(self):
        self.dev.close()


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = RandSyncNode()
    node.run()

