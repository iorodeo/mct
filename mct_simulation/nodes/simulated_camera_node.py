#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('mct_simulation')
import sys
import os
import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import mct_simulation
from mct_simulation.cad_model import CadModel


class SimulatedCamera:
  def __init__(self):
    self.initialized = False
    self.camera_number = rospy.get_param("~camera_number")

    # Test Rig
    self.cad_model = CadModel()
    self.cad_model.set_obj_parameter('display_render',False)
    self.cad_model.show_calibration(True)
    self.camera_info = self.cad_model.get_camera_info(self.camera_number)
    # self.render_dir = os.path.dirname(mct_simulation.cad_model.__file__)
    self.render_dir = os.path.expanduser('~/.ros')
    self.rendered_path = os.path.join(self.render_dir,self.camera_info['image_name'])
    # self.rendered_path = self.camera_info['image_path']

    # Broadcaster/Publishers
    self.rendered_image_pub = rospy.Publisher("/camera" + str(self.camera_number) + "/camera/image_raw",Image)
    self.rendered_image = cv.CreateImage(self.camera_info['image_size'],cv.IPL_DEPTH_8U,1)

    # OpenCV
    self.max_8U = 255
    self.color_max = 255
    self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
    self.storage = cv.CreateMemStorage()
    self.bridge = CvBridge()

    self.initialized = True

  def publish(self):
    while not rospy.is_shutdown():
      if self.initialized:
        self.cad_model.render_camera(self.camera_number)
        try:
          self.rendered_image = cv.LoadImage(self.rendered_path,cv.CV_LOAD_IMAGE_GRAYSCALE)
          self.rendered_image_pub.publish(self.bridge.cv_to_imgmsg(self.rendered_image,"passthrough"))
        except IOError:
          pass
        rospy.sleep(0.1)


if __name__ == '__main__':
  rospy.init_node('simulated_camera_node')
  sc = SimulatedCamera()
  try:
    sc.publish()
  except rospy.ROSInterruptException:
    pass
