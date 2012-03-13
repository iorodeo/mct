#!/usr/bin/env python
import roslib
roslib.load_manifest('mct_active_target')
import threading
import rospy
import time
import mct_introspection
from mct_active_target import ActiveTargetDev
from mct_utilities import file_tools

from mct_msg_and_srv.srv import ActiveTargetCmd
from mct_msg_and_srv.srv import ActiveTargetCmdResponse
from mct_msg_and_srv.srv import ActiveTargetInfo
from mct_msg_and_srv.srv import ActiveTargetInfoResponse
from mct_msg_and_srv.srv import CommandString
from mct_msg_and_srv.srv import CommandStringResponse

class ActiveTargetNode(object):

    def __init__(self):

        self.lock = threading.Lock()
        self.access_lock_owner = None
        self.sleep_dt = 1.0

        # Open device
        target_info = file_tools.read_target_info('active')
        self.dev = ActiveTargetDev(target_info)
        self.dev.off()

        rospy.init_node('active_target')
        rospy.on_shutdown(self.shutdown)

        # Setup services
        self.cmd_srv = rospy.Service(
                'active_target_cmd',
                ActiveTargetCmd,
                self.handle_cmd
                )

        self.info_srv = rospy.Service(
                'active_target_info',
                ActiveTargetInfo,
                self.handle_info
                )

        self.lock_srv = rospy.Service(
                'active_target_lock',
                CommandString,
                self.handle_lock_srv
                )

        self.lock_srv = rospy.Service(
                'active_target_unlock',
                CommandString,
                self.handle_unlock_srv
                )

    def run(self):
        rospy.spin()
        #while not rospy.is_shutdown():
        #    print(self.access_lock_owner)
        #    rospy.sleep(0.25)

    def shutdown(self):
        self.dev.off()


    def handle_lock_srv(self,req):
        """
        Try to acquire access lock
        """
        node_name = req.command
        flag = True
        message = ''
        with self.lock:
            if (self.access_lock_owner is None) or (self.access_lock_owner == node_name):
                self.access_lock_owner = node_name
            else:
                flag = False
                message = 'cannot lock - lock already owned'
        return CommandStringResponse(flag,message)


    def handle_unlock_srv(self,req):
        """
        Try to release access lock
        """
        node_name = req.command
        flag = True
        message = ''
        with self.lock:
            if self.access_lock_owner is not None:
                if node_name == self.access_lock_owner:
                    self.access_lock_owner = None
                else:
                    flag = False
                    message = 'cannot unlock - not lock owner'
        return CommandStringResponse(flag,message)


    def handle_info(self,req):
        """
        Handles request for infomartion regarding the active calibration
        target.
        """
        n = self.dev.ledArraySize[0]
        m = self.dev.ledArraySize[1]
        max_power = self.dev.maxPowerInt
        square = self.dev.square
        return ActiveTargetInfoResponse(n,m,max_power,square)

    def handle_cmd(self,req):
        """
        Handles requests for the active calibration target. Allowed commands are
        'off'       - turns off all leds
        'pattern'   - turns on the test/alignment pattern
        'led'       - turns on led specifed by given indices and the specified 
                      power level.
        """
        command = req.command.lower()
        status = True
        message = ''
        if command == 'off' or command == 'alloff':
            self.dev.off()
        elif command == 'pattern':
            self.dev.pattern()
        elif command == 'all' or command == 'allon':
            self.dev.all()
        elif command == 'led':
            if req.ind0 < 0 or req.ind0 >= self.dev.ledArraySize[0]:
                status = False
                message = 'error: led ind0 out of range'
            elif req.ind1 < 0 or req.ind1 >= self.dev.ledArraySize[1]:
                status = False
                message = 'error: led ind1 out of range'
            elif req.power < 0 or req.power >= self.dev.maxPowerInt:
                status = False
                message = 'error: power level out of range'
            else:
                self.dev.led(req.ind0, req.ind1, req.power)
        else:
            status = False
            message = 'error: unkown command: {0}'.format(command)
        return ActiveTargetCmdResponse(status,message)



# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = ActiveTargetNode()
    node.run()




