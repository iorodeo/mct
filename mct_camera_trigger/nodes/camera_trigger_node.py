#!/usr/bin/env python
import roslib
roslib.load_manifest('mct_camera_trigger')
import rospy
import time
from mct_camera_trigger.trigger_device import CamTrigDev

from mct_msg_and_srv.srv import CameraTriggerCmd
from mct_msg_and_srv.srv import CameraTriggerCmdResponse

OPEN_SLEEP_DT = 1.0


class TriggerNode(object):
    """
    Node for communicating with the camera trigger device. Provides services
    for start camera trigger pulses and the requested frequency and stopping
    trigger pulses.
    """

    def __init__(self):

        # Get device parameters
        self.port = rospy.get_param('/camera_trigger/port','/dev/camera-trigger')
        self.baudrate = rospy.get_param('/camera_trigger/baudrate', 115200)

        # Open device
        self.dev = CamTrigDev(port=self.port,baudrate=self.baudrate)
        time.sleep(OPEN_SLEEP_DT) # Slight delay for opening device
        self.dev.stop()

        # Initialize node and setup trigger command service
        rospy.init_node('camera_trigger_device')
        rospy.on_shutdown(self.on_shutdown)
        self.trigger_cmd_srv = rospy.Service(
                'camera_trigger_cmd',
                CameraTriggerCmd,
                self.handle_trigger_cmd
                )

    def on_shutdown(self):
        """
        On shutdown start triggers running at some base rate to ensure that the 
        camera nodes w/ external triggers can shutdown properly. 
        """
        self.dev.start(10)

    def handle_trigger_cmd(self,req):
        """
        Handles request to start and stop camera trigger pulses.
        """
        command = req.command.lower()
        flag = True
        if command == 'start':
            freq = req.frequency
            self.dev.stop()
            self.dev.start(freq)
        elif command == 'stop':
            self.dev.stop()
        else:
            flag = False
        return CameraTriggerCmdResponse(flag)


    def run(self):
        rospy.spin()


# ---------------------------------------------------------------------------------------
if __name__ == '__main__':

    node = TriggerNode()
    node.run()


