#!/usr/bin/python
from __future__ import division
import roslib
roslib.load_manifest('mct_simulation')
import rospy

def find_simulated_camera_images():
    """
    Returns a list of all active simulated camera images by looking for topics of the form
    /simulation/camera_name/camera/image_raw
    """
    simulated_camera_images = set()
    topic_list = rospy.get_published_topics('/')
    for topic in topic_list:
        topic = topic[0]
        topic_split = topic.split('/')
        if len(topic_split) <= 4:
            continue
        if (topic_split[1] == 'simulation') and (topic_split[4] == 'image_raw'):
            simulated_camera_image_name =topic
            simulated_camera_images.add(simulated_camera_image_name)
    simulated_camera_images = list(simulated_camera_images)
    simulated_camera_images.sort()
    return simulated_camera_images
