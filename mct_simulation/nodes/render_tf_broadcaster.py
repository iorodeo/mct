#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('mct_simulation')
import rospy
import tf
import random

import mct_simulation
from mct_simulation.cad_model import CadModel


class RenderTfBroadcaster:
  def __init__(self):
    self.initialized = False

    self.camera_name = rospy.get_param("~camera_name")

    self.rate = rospy.Rate(1)

    # CAD Model
    self.cad_model = CadModel()
    self.cad_model.set_camera(self.camera_name)

    self.camera_projection = self.cad_model.camera.get_obj_parameter('camera_projection')
    self.camera_angle = self.cad_model.camera.get_obj_parameter('camera_angle')
    self.camera_position = self.cad_model.camera.get_position()
    self.camera_look_at = self.cad_model.camera.get_obj_parameter('camera_look_at')
    self.image_size = self.cad_model.camera.get_obj_parameter('image_size')

    # Broadcaster/Publishers
    self.br = tf.TransformBroadcaster()

    self.initialized = True

  def find_position_orientation(self):
    pos_x = self.camera_look_at[0] + random.uniform(-.1,.1)
    pos_y = self.camera_look_at[1] + random.uniform(-.1,.1)
    pos_z = self.camera_look_at[2] + random.uniform(-.1,.1)
    # pos_x = self.camera_look_at[0]
    # pos_y = self.camera_look_at[1]
    # pos_z = self.camera_look_at[2]
    position = [pos_x,pos_y,pos_z]
    orientation = tf.transformations.quaternion_from_euler(0, 0, 0)
    return position,orientation

  def publish(self):
    while not rospy.is_shutdown():
      if self.initialized:
        position,orientation = self.find_position_orientation()
        self.br.sendTransform(position,
                              orientation,
                              rospy.Time.now(),
                              "render_object",
                              "world")
        self.rate.sleep()


if __name__ == '__main__':
  rospy.init_node('render_tf_broadcaster')
  rtb = RenderTfBroadcaster()
  try:
    rtb.publish()
  except rospy.ROSInterruptException:
    pass
