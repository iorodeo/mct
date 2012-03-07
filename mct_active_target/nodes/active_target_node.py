#!/usr/bin/env python
import roslib
roslib.load_manifest('mct_active_target')
import rospy
import time
from mct_active_target import ActiveTargetDev

from mct_msg_and_srv.srv import ActiveTargetCmd
from mct_msg_and_srv.srv import ActiveTargetCmdResponse
from mct_msg_and_srv.srv import ActiveTargetInfo
from mct_msg_and_srv.srv import ActiveTargetInfoResponse

class ActiveTargetNode(object):

    def __init__(self):

        # Get device parameters
        self.port = rospy.get_param('active_target_port','/dev/ttyUSB0')
        self.baudrate = rospy.get_param('active_target_baudrate', 9600)

        # Open device
        self.dev = ActiveTargetDev(port=self.port,baudrate=self.baudrate)
        self.dev.off()

        rospy.init_node('active_target')

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

    def handle_info(self,req):
        """
        Handles request for infomartion regarding the active calibration
        target.
        """
        n = self.dev.ledArraySize[0]
        m = self.dev.ledArraySize[1]
        max_power = self.dev.maxPowerInt
        return ActiveTargetInfoResponse(n,m,max_power)

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
        if command == 'off':
            self.dev.off()
        elif command == 'pattern':
            self.dev.pattern()
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

    def run(self):
        rospy.spin()


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = ActiveTargetNode()
    node.run()




