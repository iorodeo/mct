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

# Units of meters
PARAMETERS = {
    # 'beam_side_length' : 1.5,
    # 'box_side_length' : 94.5,
    'beam_side_length' : 0.0381,
    'box_side_length' : 2.4,
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

    def render(self,position=[0,0,0],orientation=[0,0,0,1]):
        if self.show_box_bool and (not self.show_calibration_bool):
            self.set_obj_list(self.box)
        else:
            self.set_obj_list([])
        if self.show_floor_bool:
            self.add_obj(self.floor_checkerboard)
        if self.show_calibration_bool:
            self.__place_calibration_checkerboard(position,orientation)
            self.add_obj(self.calibration_checkerboard)
        super(CadModel, self).render()

    def __set_light_sources(self):
        self.add_light(position=[-3000,3000,-3000])
        # self.add_light(position=[3000,-3000,-3000])

    def __add_cameras(self):
        regular_angle = 65*math.pi/180
        fisheye_angle = 185*math.pi/180
        # camera_z = 80
        camera_z = 2.032
        floor_z = 0
        image_size = [640,480]
        # image_dir = '~/.multi_cam_tracker'
        image_dir = ''
        # self.add_camera('sim_camera1','perspective',regular_angle,[19,-42.75,camera_z],[19,-42.75,floor_z],image_size,image_dir)
        # self.add_camera('sim_camera2','perspective',regular_angle,[-19,-42.75,camera_z],[-19,-42.75,floor_z],image_size,image_dir)
        # self.add_camera('sim_camera3','perspective',regular_angle,[38,-14.25,camera_z],[38,-14.25,floor_z],image_size,image_dir)
        # self.add_camera('sim_camera4','perspective',regular_angle,[0,-14.25,camera_z],[0,-14.25,floor_z],image_size,image_dir)
        # self.add_camera('sim_camera5','perspective',regular_angle,[-38,-14.25,camera_z],[-38,-14.25,floor_z],image_size,image_dir)
        # self.add_camera('sim_camera6','fisheye',fisheye_angle,[0,0,camera_z],[0,0,floor_z],image_size,image_dir)
        # self.add_camera('sim_camera7','perspective',regular_angle,[38,14.25,camera_z],[38,14.25,floor_z],image_size,image_dir)
        # self.add_camera('sim_camera8','perspective',regular_angle,[0,14.25,camera_z],[0,14.25,floor_z],image_size,image_dir)
        # self.add_camera('sim_camera9','perspective',regular_angle,[-38,14.25,camera_z],[-38,14.25,floor_z],image_size,image_dir)
        # self.add_camera('sim_camera10','perspective',regular_angle,[19,42.75,camera_z],[19,42.75,floor_z],image_size,image_dir)
        # self.add_camera('sim_camera11','perspective',regular_angle,[-19,42.75,camera_z],[-19,42.75,floor_z],image_size,image_dir)

        self.add_camera('sim_camera1','perspective',regular_angle,[0.4826,-1.08585,camera_z],[0.4826,-1.08585,floor_z],image_size,image_dir)
        self.add_camera('sim_camera2','perspective',regular_angle,[-0.4826,-1.08585,camera_z],[-0.4826,-1.08585,floor_z],image_size,image_dir)
        self.add_camera('sim_camera3','perspective',regular_angle,[0.9652,-0.36195,camera_z],[0.9652,-0.36195,floor_z],image_size,image_dir)
        self.add_camera('sim_camera4','perspective',regular_angle,[0,-0.36195,camera_z],[0,-0.36195,floor_z],image_size,image_dir)
        self.add_camera('sim_camera5','perspective',regular_angle,[-0.9652,-0.36195,camera_z],[-0.9652,-0.36195,floor_z],image_size,image_dir)
        self.add_camera('sim_camera6','fisheye',fisheye_angle,[0,0,camera_z],[0,0,floor_z],image_size,image_dir)
        self.add_camera('sim_camera7','perspective',regular_angle,[0.9652,0.36195,camera_z],[0.9652,0.36195,floor_z],image_size,image_dir)
        self.add_camera('sim_camera8','perspective',regular_angle,[0,0.36195,camera_z],[0,0.36195,floor_z],image_size,image_dir)
        self.add_camera('sim_camera9','perspective',regular_angle,[-0.9652,0.36195,camera_z],[-0.9652,0.36195,floor_z],image_size,image_dir)
        self.add_camera('sim_camera10','perspective',regular_angle,[0.4826,1.08585,camera_z],[0.4826,1.08585,floor_z],image_size,image_dir)
        self.add_camera('sim_camera11','perspective',regular_angle,[-0.4826,1.08585,camera_z],[-0.4826,1.08585,floor_z],image_size,image_dir)

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
        self.floor_checkerboard = Checkerboard(0.254,8,8)
        # self.add_obj(self.floor_checkerboard)

    def __make_calibration_checkerboard(self):
        checker_size = 0.127
        checker_count_x = 8
        checker_count_y = 6
        self.calibration_checkerboard = Checkerboard(checker_size,checker_count_x,checker_count_y)
        self.calibration_checkerboard_size = [checker_size*checker_count_x,checker_size*checker_count_y]

    def get_calibration_checkerboard_size(self):
        return copy.deepcopy(self.calibration_checkerboard_size)

    def __place_calibration_checkerboard(self,position=[0,0,0],orientation=[0,0,0,1]):
        self.calibration_checkerboard.set_position(position)
        self.calibration_checkerboard.set_orientation(orientation)


# ---------------------------------------------------------------------
if __name__ == '__main__':
    cad_model = CadModel()
    cad_model.render_all()
