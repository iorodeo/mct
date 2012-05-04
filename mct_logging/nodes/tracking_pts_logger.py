#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_logging')
import rospy
import threading
import yaml
import json

import mct_msg_and_srv.msg
from mct_msg_and_srv.srv import LoggingCmd
from mct_msg_and_srv.srv import LoggingCmdResponse

class TrackingPtsLogger(object):

    """
    Simple node for logging the tracking points data. Logs data to json format. Doesn't 
    need to know the topic type. However, the topic type must be in mct_msg_and_srv.msg.
    """

    def __init__(self,topic):

        self.ready = False
        self.logging = False
        self.fid = None
        self.count = 0
        self.lock = threading.Lock()
        rospy.init_node('three_points_logger')
        topic_type = self.get_topic_type(topic)
        self.node_name = rospy.get_name()


        # Subscribe to topic
        self.topic_sub = rospy.Subscriber(topic,topic_type,self.topic_sub_handler)

        # Setup start/stop service
        self.logging_srv = rospy.Service(
                '{0}/logging_cmd'.format(self.node_name), 
                LoggingCmd, 
                self.logging_srv_handler
                )

        self.ready = True

    def get_topic_type(self,topic):
        """
        Automatically determines topic type
        """
        topic_list = rospy.get_published_topics()
        topic_dict = dict(topic_list)
        topic_name= topic_dict[topic]
        topic_name = topic_name.split('/')[-1]
        topic_type = getattr(mct_msg_and_srv.msg,topic_name)
        return topic_type


    def logging_srv_handler(self,req):
        """
        Handles requests to start and stop the logging services.
        """

        with self.lock:
            cmd = req.command.lower()
            filename = req.filename
            flag = True
            if cmd == 'start':
                if not self.logging:
                    self.logging = True
                    self.fid = open(filename,'w')
                    self.fid.write('[\n')
                    self.count = 0
                else:
                    flag = False
            elif cmd == 'stop':
                if self.logging:
                    self.fid.write('\n]\n')
                    self.fid.close()
                    self.fid = None
                    self.logging = False
                else:
                    flag = False
            else:
                flag = False
        return LoggingCmdResponse(flag)

    def topic_sub_handler(self,data):
        """
        Receives messages and writes them to file if logging is turned on.
        """
        if not self.ready:
            return
        with self.lock:
            if self.logging:
                if self.count > 0:
                    self.fid.write(',\n')
                data_dict = yaml.load(str(data))
                json.dump(data_dict, self.fid)
                self.count += 1

    def run(self):
        rospy.spin()


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import sys
    topic = sys.argv[1]

    node = TrackingPtsLogger(topic)
    node.run()


