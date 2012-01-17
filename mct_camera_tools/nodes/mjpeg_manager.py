#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
import os
import os.path
import tempfile
import subprocess
import json
from mct_introspection import find_camera_topics
from mct_xml_tools.launch import create_mjpeg_server_launch

# Services
from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import CommandStringResponse
from mct_msg_and_srv.srv import GetJSONString
from mct_msg_and_srv.srv import GetJSONStringResponse

class MJPEG_Manager(object):

    def __init__(self):

        self.server_running = False
        self.mjpeg_info_dict = None
        self.tmp_dir = tempfile.gettempdir()
        self.server_launch_file = os.path.join(self.tmp_dir,'mjpeg_server.launch')
        self.server_popen = None
        self.mjpeg_start_port = rospy.get_param('mjpeg_start_port',8080)

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('mjpeg_manager')

        self.mjpeg_servers_srv = rospy.Service(
                'mjpeg_servers',
                CommandString, 
                self.handle_mjpeg_servers_srv
                )
        self.mjpeg_servers_info_srv = rospy.Service(
                'mjpeg_servers_info',
                GetJSONString,
                self.handle_mjpeg_servers_info_srv
                )

    def run(self):
        rospy.spin()

    def handle_mjpeg_servers_info_srv(self,req):
        """
        Handles request for information about the mjpeg servers. Returns a python
        dictionary - serialized as a json string - containing the mjpeg_info_dict 
        dictionary.
        """
        if self.mjpeg_info_dict == None:
            mjpeg_info_json = json.dumps({})
        else:
            mjpeg_info_json = json.dumps(self.mjpeg_info_dict)
        return GetJSONStringResponse(mjpeg_info_json)

    def handle_mjpeg_servers_srv(self,req):
        """
        Handles request for starting and stoping the mjpeg servers. Creates a dictionary
        of mjpeg server information.
        """
        cmd = req.command
        cmd = cmd.lower()
        response = True
        message = ''
        if cmd == 'start':
            # Start mjpeg servers if they haven't already been started
            if not self.server_running:
                self.launch_mjpeg_servers()
            else:
                response = False
                message = 'mjpeg servers already running'

        elif cmd == 'stop':
            # Stop mjpeg servers if they are running
            if self.server_running:
                self.kill_mjpeg_servers()
                self.delete_server_launch_file()
            else:
                repsonse = False
                message = 'mjpeg servers not running'
        else:
            response = False
            message = 'unknown command {0}'.format(cmd)
        return CommandStringResponse(response,message)

    def launch_mjpeg_servers(self):
        """
        Creates the mjpeg server launch file and launches the mjpeg server nodes.
        """
        camera_topics = find_camera_topics()
        self.mjpeg_info_dict = self.create_mjpeg_info_dict(camera_topics)
        create_mjpeg_server_launch(self.server_launch_file,self.mjpeg_info_dict)
        self.server_popen = subprocess.Popen(['roslaunch',self.server_launch_file])
        self.server_running = True

    def kill_mjpeg_servers(self):
        """
        Kills the mjpeg server nods by killing the launch process.
        """
        if self.server_popen is not None:
            self.server_popen.send_signal(subprocess.signal.SIGINT)
            self.server_popen = None
        self.mjpeg_info_dict = None
        self.server_running = False

    def delete_server_launch_file(self):
        """
        Deletes the mjpeg server temporary launch file.
        """
        if os.path.isfile(self.server_launch_file):
            os.remove(self.server_launch_file)

    def create_mjpeg_info_dict(self, camera_topics):
        """
        Creates the mjpeg server information dictionary which gives a mapping
        from the camera name to a dictionary contaiing the camera topic, the
        mjpeg server name, and the mjpeg server port number.
        """
        mjpeg_info_dict = {}
        for i, topic in enumerate(camera_topics):
            camera_name = topic.split('/')[2]
            mjpeg_server_name = 'mjpeg_server_{0}'.format(camera_name)
            mjpeg_server_port = self.mjpeg_start_port + i
            info = {
                    'caemra_topic' : topic,
                    'mjpeg_server' : mjpeg_server_name,
                    'mjpeg_port'   : mjpeg_server_port,
                    }
            mjpeg_info_dict[camera_name] = info 
        return mjpeg_info_dict

    def clean_up(self):
        """
        Clean up function for node shutdown.
        """
        self.kill_mjpeg_servers()
        self.delete_server_launch_file()




# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = MJPEG_Manager()
    node.run()
