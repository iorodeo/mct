#!/usr/bin/env python
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy

# Services
from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import CommandStringResponse

class MJPEG_Manager(object):

    def __init__(self):

        self.servers_running = False
        rospy.on_shutdown(self.clean_up)
        rospy.init_node('mjpeg_manager')

        self.mjpeg_srv = rospy.Service(
                'mjpeg_manager',
                CommandString, 
                self.handle_mjpeg_srv
                )

    def run(self):
        rospy.spin()

    def handle_mjpeg_srv(self,req):
        cmd = req.command
        cmd = cmd.lower()
        response = True
        message = ''
        if cmd == 'start':
            # Start mjpeg servers if they haven't already been started
            if not servers_running:
                pass
            else:
                response = False
                message = 'mjpeg servers already running'

        elif cmd = 'stop':
            # Stop mjpeg servers if they are running
            if servers_running:
                pass
            else:
                repsonse = False
                message = 'mjpeg servers not running'
        else:
            response = False
            message = 'unknown command {0}'.format(cmd)
        return CommandStringResponse(response,message)

    def clean_up(self):
        pass

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = MJPEG_Manager()
    node.run()
