#!/usr/bin/env python
import roslib 
roslib.load_manifest('mct_camera_tools')
import rospy
import subprocess
import signal
import os
from find_camera_topics import find_camera_topics

PORT_DICT = { 
        'image_raw' : (8080,8181),
        'image_rect': (8282,8383),
        'other'     : (8484,8585),
        }

class MJPEG_Streamer(object):

    def __init__(self,image='image_raw',rate=10.0):
        self.proc_list = []
        self.image = image
        self.rate = rate
        self.ports = PORT_DICT


    def start(self):
        """
        Starts throttling nodes and mjped camera streams.
        """
        env = os.environ.copy()
        # Start camera image throttling nodes
        camera_list = find_camera_topics()
        for camera in camera_list:
            env['ROS_NAMESPACE'] = '%s'%(camera,)
            proc_cmd = [ 
                    'rosrun',
                    'topic_tools',
                    'throttle',
                    'messages',
                    '/%s/camera/%s'%(camera,self.image,),
                    str(self.rate),
                    '/%s/camera/%s_throttle'%(camera,self.image,),
                    '__name:=%s_throttler'%(self.image,),
                    ]
            proc = subprocess.Popen(proc_cmd, env=env)
            self.proc_list.append(proc)
        del env['ROS_NAMESPACE']

        # Start mjpeg servers - really need to determin number of servers based on the
        # number of cameras
        #
        # It looks like we may want one port per image ...
        try:
            server_ports = self.ports[self.image]
        except KeyError:
            server_ports = self.ports['other']

        for port in server_ports:
            proc_cmd = [
                    'rosrun', 
                    'mjpeg_server',
                    'mjpeg_server',
                    '__name:=mjpeg_server%s'%(port,),
                    '_port:=%s'%(port,),
                    ]
            proc = subprocess.Popen(proc_cmd, env=env)
            self.proc_list.append(proc)

    def stop(self):
        for proc in self.proc_list:
            proc.send_signal(signal.SIGINT)

def get_camera_ports(camera_list,image='image_raw'):
    """
    Cheesey temporary function to allocate ports to camera streams. 
    """
    port_list = []
    for i, camera in enumerate(camera_list):
        if i <= 3:
            port_list.append(PORT_DICT[image][0])
        else:
            port_list.append(PORT_DICT[image][1])
    return port_list

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Simple test of MJPEG_Streamer class
    import time
    streamer = MJPEG_Streamer()
    streamer.start()
    time.sleep(4.0)
    print 
    raw_input('Press enter to exit')
    print 
    streamer.stop()
    


