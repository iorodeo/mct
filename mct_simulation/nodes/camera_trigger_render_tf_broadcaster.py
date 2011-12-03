#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('mct_simulation')
import rospy
import tf
import random
import numpy
import math
from std_msgs.msg import String

import mct_simulation
from mct_simulation.cad_model import CadModel


class RenderTfBroadcaster:
  def __init__(self):
    self.initialized = False

    self.camera_name = rospy.get_param("~camera_name")

    self.dev_small = True

    self.trigger_rate = rospy.Rate(2)
    if self.dev_small:
      self.trigger_render_ratio = 200
    else:
      self.trigger_render_ratio = 20
    self.trigger_count = 0

    # CAD Model
    self.cad_model = CadModel()
    self.cad_model.set_camera(self.camera_name)

    self.camera_projection = self.cad_model.camera.get_obj_parameter('camera_projection')
    self.camera_angle = self.cad_model.camera.get_obj_parameter('camera_angle')
    self.camera_position = self.cad_model.camera.get_position()
    self.camera_look_at = self.cad_model.camera.get_obj_parameter('camera_look_at')
    self.image_size = self.cad_model.camera.get_obj_parameter('image_size')

    self.calibration_checkerboard_size = self.cad_model.get_calibration_checkerboard_size()

    # Broadcaster/Publishers
    self.render_tf_br = tf.TransformBroadcaster()
    self.camera_trigger_pub = rospy.Publisher('/camera/trigger', String)

    self.tf_initialized = False
    self.initialized = True

  def find_position_orientation(self):
    camera_position = numpy.array(self.camera_position)
    camera_look_at = numpy.array(self.camera_look_at)
    camera_dir =  camera_look_at - camera_position
    camera_dist = numpy.linalg.norm(camera_dir)
    camera_dir /= camera_dist
    camera_angle = min([self.camera_angle,7*math.pi/8])

    if self.dev_small:
      dist_min = max(self.calibration_checkerboard_size)
    else:
      dist_min = max(self.calibration_checkerboard_size)/4
    dist_max = camera_dist

    dist = random.uniform(dist_min,dist_max)

    position = camera_position + camera_dir*dist

    dev = dist*math.tan(camera_angle/2)
    if self.dev_small:
      dev /= 8
    else:
      dev /= 2

    position[0] += random.uniform(-dev,dev)
    position[1] += random.uniform(-dev,dev)

    if self.dev_small:
      angle_min = -15*math.pi/180
      angle_max = 15*math.pi/180
    else:
      angle_min = -45*math.pi/180
      angle_max = 45*math.pi/180

    ang_x = random.uniform(angle_min,angle_max)
    ang_y = random.uniform(angle_min,angle_max)
    orientation = tf.transformations.quaternion_from_euler(ang_x,ang_y, 0)

    # position = camera_look_at
    # orientation = [0,0,0,1]

    return position,orientation

  def publish(self):
    while not rospy.is_shutdown():
      if self.initialized:
        if (self.trigger_count < self.trigger_render_ratio) and self.tf_initialized:
          self.trigger_count += 1
        else:
          if not self.tf_initialized:
            self.tf_initialized = True
          self.trigger_count = 0
          position,orientation = self.find_position_orientation()
          self.render_tf_br.sendTransform(position,
                                          orientation,
                                          rospy.Time.now(),
                                          "render_object",
                                          "world")
        self.camera_trigger_pub.publish(String(""))
        self.trigger_rate.sleep()


if __name__ == '__main__':
  rospy.init_node('camera_trigger_render_tf_broadcaster')
  rtb = RenderTfBroadcaster()
  try:
    rtb.publish()
  except rospy.ROSInterruptException:
    pass
