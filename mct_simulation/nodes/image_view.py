#!/usr/bin/python
from __future__ import division
import roslib
roslib.load_manifest('mct_simulation')
import rospy
import subprocess
import signal
import time
import os


def find_rendered_images():
    """
    Returns a list of all active rendered_images by looking for topics of the form
    /cameraN/camera/rendered
    """
    rendered_images = set()
    topic_list = rospy.get_published_topics('/')
    for topic in topic_list:
        topic = topic[0]
        topic_split = topic.split('/')
        if len(topic_split) <= 3:
            continue
        if topic_split[3] == 'rendered':
            rendered_image_name =topic
            rendered_images.add(rendered_image_name)
    rendered_images = list(rendered_images)
    rendered_images.sort()
    return rendered_images


if __name__ == "__main__":
    env = os.environ.copy()
    #env['ROS_MASTER_URI'] = 'http://felis:11311'
    rendered_images = find_rendered_images()
    proc_list = []
    for rendered_image in rendered_images:
        proc_list.append(subprocess.Popen(['rosrun','image_view','image_view','image:=' + rendered_image,'raw','&'],env=env))

    raw_input('Press Enter to exit.')
    for proc in proc_list:
        proc.send_signal(signal.SIGINT)
