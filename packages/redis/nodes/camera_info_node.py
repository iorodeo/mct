#!/usr/bin/env python
import roslib
roslib.load_manifest('redis')
import rospy
import redis
import threading
import cPickle as pickle
from tracker_camera_tools import find_cameras

# Messages
from sensor_msgs.msg import CameraInfo

REDIS_DB_NUMBER = 2

class Redis_Camera_Info(object):
    """
    Dumps camera info messages into the redis database
    """

    def __init__(self):
        self.db = redis.Redis('localhost', db=REDIS_DB_NUMBER)
        self.camera_list = find_cameras() 
        rospy.init_node('redis_camera_info')
        rospy.on_shutdown(self.clean_up)
        self.subscriber_list = []
        self.handler_list = []
        for camera in self.camera_list:
            topic = "/%s/camera/camera_info"%(camera,)
            handler = self.get_handler(camera)
            subscriber = rospy.Subscriber(topic,CameraInfo,handler)
            self.subscriber_list.append(subscriber)
            self.handler_list.append(handler)
        rospy.spin()

    def get_handler(self,camera):
        key = self.get_key(camera)
        def handler(data):
            data_str = pickle.dumps(data)
            self.db.set(key,data_str)
        return handler

    def get_key(self,camera):
        return '%s_info'%(camera,)

    def clean_up(self):
        for camera in self.camera_list:
            key = self.get_key(camera)
            self.db.delete(key)
           
# -----------------------------------------------------------------------------
if __name__ == '__main__':
    node = Redis_Camera_Info()



