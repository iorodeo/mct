#!/usr/bin/env python
import roslib 
roslib.load_manifest('tracker_camera_tools')
import rospy

def find_cameras():
    """
    Returns a list of all active cameras by looking for topics of the form 
    /cameraN/camera
    """
    cameras = set() 
    topic_list = rospy.get_published_topics('/')
    for topic in topic_list:
        topic= topic[0]
        topic = topic.split('/')
        if len(topic) <= 2:
            continue
        if topic[2] == 'camera':
            camera_name =topic[1]
            cameras.add(camera_name)
    cameras = list(cameras)
    cameras.sort()
    return cameras

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    cameras = find_cameras()
    for name in cameras:
        print name
