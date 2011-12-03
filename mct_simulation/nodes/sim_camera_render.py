#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('mct_simulation')
import sys
import os
import rospy
import cv
import tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

import mct_simulation
from mct_simulation.cad_model import CadModel


class SimCameraRender:
  def __init__(self):
    self.initialized = False
    self.camera_name = rospy.get_param("~camera_name")

    # CAD Model
    self.cad_model = CadModel()
    self.cad_model.set_obj_parameter('display_render',False)
    self.cad_model.show_calibration(True)
    self.cad_model.set_camera(self.camera_name)
    # self.render_dir = os.path.dirname(mct_simulation.cad_model.__file__)
    self.render_dir = os.path.expanduser('~/.ros')
    self.rendered_path = os.path.join(self.render_dir,self.cad_model.camera.get_obj_parameter('image_name'))

    self.rendered_image = cv.CreateImage(self.cad_model.camera.get_obj_parameter('image_size'),cv.IPL_DEPTH_8U,3)

    # Broadcaster/Publishers
    self.rendered_image_pub = rospy.Publisher("/" + self.camera_name + "/camera/rendered",Image)

    # Listeners/Subscribers
    self.tf_listener = tf.TransformListener()
    rospy.Subscriber("/camera/trigger", String, self.publish)

    # OpenCV
    self.max_8U = 255
    self.color_max = 255
    self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
    self.storage = cv.CreateMemStorage()
    self.bridge = CvBridge()

    self.initialized = True

  def publish(self,data):
    if self.initialized:
      try:
        (position,orientation) = self.tf_listener.lookupTransform('world','render_object',rospy.Time(0))
        self.cad_model.render(position,orientation)
        self.rendered_image = cv.LoadImage(self.rendered_path,cv.CV_LOAD_IMAGE_COLOR)
        self.rendered_image_pub.publish(self.bridge.cv_to_imgmsg(self.rendered_image,"bgr8"))
      except (tf.LookupException, tf.ConnectivityException, IOError, CvBridgeError):
        pass


if __name__ == '__main__':
  rospy.init_node('sim_camera_render')
  scr = SimCameraRender()
  while not rospy.is_shutdown():
    rospy.spin()
