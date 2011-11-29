#!/usr/bin/python
from __future__ import division
import roslib
roslib.load_manifest('mct_simulation')
import rospy

def find_simulated_camera_images():
    """
    Returns a list of all active simulated camera images by looking for topics of the form
    /camera_name/camera/image_raw
    """
    simulated_camera_images = set()
    topic_list = rospy.get_published_topics('/')
    for topic in topic_list:
        topic = topic[0]
        topic_split = topic.split('/')
        if len(topic_split) <= 2:
            continue
        if (topic_split[3] == 'image_raw'):
            name = topic_split[1]
            name_split = name.split('_')
            if name_split[0] == "sim":
                simulated_camera_image_name = topic
                simulated_camera_images.add(simulated_camera_image_name)
    simulated_camera_images = list(simulated_camera_images)
    simulated_camera_images.sort()
    return simulated_camera_images

def find_camera_name(simulated_camera_image):
    topic_split = simulated_camera_image.split('/')
    return '/' + topic_split[1] + '/' + topic_split[2]
