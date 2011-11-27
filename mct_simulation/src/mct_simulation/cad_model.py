from __future__ import division
import roslib
roslib.load_manifest('mct_simulation')
import rospy

import copy
import os
import math
import random

import cad.finite_solid_objects as fso
import cad.csg_objects as csg
import cad.pattern_objects as po

from checkerboard import Checkerboard

# Units of inches, scale later to be in mm
PARAMETERS = {
    'beam_side_length' : 1.5,
    'box_side_length' : 94.5,
    'box_color' : [0.8,0.8,0.8,1.0],
    }


class CadModel(csg.Union):
    def __init__(self):
        super(CadModel, self).__init__()
        self.update_obj_parameters(PARAMETERS)
        self.__set_light_sources()
        self.__add_cameras()
        self.__make_box()
        self.__make_floor_checkerboard()
        self.__make_calibration_checkerboard()
        self.show_box_bool = True
        self.show_floor_bool = False
        self.show_calibration_bool = False

    def show_box(self,show_box_bool=True):
        self.show_box_bool = bool(show_box_bool)

    def show_floor(self,show_floor_bool=True):
        self.show_floor_bool = bool(show_floor_bool)

    def show_calibration(self,show_calibration_bool=True):
        self.show_calibration_bool = bool(show_calibration_bool)

    def render(self):
        if self.show_box_bool and (not self.show_calibration_bool):
            self.set_obj_list(self.box)
        else:
            self.set_obj_list([])
        if self.show_floor_bool:
            self.add_obj(self.floor_checkerboard)
        if self.show_calibration_bool:
            self.__place_calibration_checkerboard()
            self.add_obj(self.calibration_checkerboard)
        super(CadModel, self).render()

    def __set_light_sources(self):
        self.add_light(position=[-3000,3000,-3000])
        # self.add_light(position=[3000,-3000,-3000])

    def __add_cameras(self):
        regular_angle = 65
        fisheye_angle = 185
        camera_z = 80
        floor_z = 0
        image_size = [640,480]
        # image_dir = '~/.multi_cam_tracker'
        image_dir = ''
        self.add_camera('camera1','perspective',regular_angle,[19,-42.75,camera_z],[19,-42.75,floor_z],image_size,image_dir)
        self.add_camera('camera2','perspective',regular_angle,[-19,-42.75,camera_z],[-19,-42.75,floor_z],image_size,image_dir)
        self.add_camera('camera3','perspective',regular_angle,[38,-14.25,camera_z],[38,-14.25,floor_z],image_size,image_dir)
        self.add_camera('camera4','perspective',regular_angle,[0,-14.25,camera_z],[0,-14.25,floor_z],image_size,image_dir)
        self.add_camera('camera5','perspective',regular_angle,[-38,-14.25,camera_z],[-38,-14.25,floor_z],image_size,image_dir)
        self.add_camera('camera6','fisheye',fisheye_angle,[0,0,camera_z],[0,0,floor_z],image_size,image_dir)
        self.add_camera('camera7','perspective',regular_angle,[38,14.25,camera_z],[38,14.25,floor_z],image_size,image_dir)
        self.add_camera('camera8','perspective',regular_angle,[0,14.25,camera_z],[0,14.25,floor_z],image_size,image_dir)
        self.add_camera('camera9','perspective',regular_angle,[-38,14.25,camera_z],[-38,14.25,floor_z],image_size,image_dir)
        self.add_camera('camera10','perspective',regular_angle,[19,42.75,camera_z],[19,42.75,floor_z],image_size,image_dir)
        self.add_camera('camera11','perspective',regular_angle,[-19,42.75,camera_z],[-19,42.75,floor_z],image_size,image_dir)

    def __make_box(self):
        beam_sl = self.get_obj_parameter('beam_side_length')
        box_sl = self.get_obj_parameter('box_side_length')
        box_color = self.get_obj_parameter('box_color')
        beam_length = box_sl - beam_sl
        beam_x = fso.Box(x=beam_length,y=beam_sl,z=beam_sl)
        beams_x = po.LinearArray(beam_x,x=[0],y=[-box_sl/2,box_sl/2],z=0)
        beam_y = fso.Box(x=beam_sl,y=beam_length,z=beam_sl)
        beams_y = po.LinearArray(beam_y,x=[-box_sl/2,box_sl/2],y=0,z=0)
        beam_z = fso.Box(x=beam_sl,y=beam_sl,z=beam_length)
        beams_z = po.LinearArray(beam_z,x=[-box_sl/2,box_sl/2],y=[-box_sl/2,box_sl/2],z=beam_length/2)
        self.box = beams_x | beams_y | beams_z
        self.box.set_color(box_color,recursive=True)
        # self.add_obj(self.box)

    def __make_floor_checkerboard(self):
        self.floor_checkerboard = Checkerboard(10,8,8)
        # self.add_obj(self.floor_checkerboard)

    def __make_calibration_checkerboard(self):
        self.calibration_checkerboard = Checkerboard(5,8,6)

    def __place_calibration_checkerboard(self):
        projection = self.camera.get_obj_parameter('camera_projection')
        angle = self.camera.get_obj_parameter('camera_angle')
        position = self.camera.get_position()
        look_at = self.camera.get_obj_parameter('camera_look_at')
        image_size = self.camera.get_obj_parameter('image_size')

        midpoint = [(position[0]+look_at[0])/2,(position[1]+look_at[1])/2,(position[2]+look_at[2])/2]
        self.calibration_checkerboard.set_orientation()

        angle_max_deg = 45
        angle_min_deg = -45
        rand_angle_x = random.randrange(angle_min_deg,angle_max_deg)*math.pi/180
        rand_angle_y = random.randrange(angle_min_deg,angle_max_deg)*math.pi/180
        self.calibration_checkerboard.rotate(angle=rand_angle_x,axis=[1,0,0])
        self.calibration_checkerboard.rotate(angle=rand_angle_y,axis=[0,1,0])

        dev_max = 25
        x_pos = midpoint[0] + random.randrange(-dev_max,dev_max)
        y_pos = midpoint[1] + random.randrange(-dev_max,dev_max)
        z_pos = random.randrange((look_at[2]+midpoint[2])/2,(position[2]+midpoint[2])/2)
        self.calibration_checkerboard.set_position([x_pos,y_pos,z_pos])


# ---------------------------------------------------------------------
if __name__ == '__main__':
    cad_model = CadModel()
    cad_model.render_all()
