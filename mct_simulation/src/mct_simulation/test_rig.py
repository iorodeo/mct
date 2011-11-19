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
    'color' : [0.8,0.8,0.8,1.0],
    'name' : 'TEST_RIG',
    'description' : '',
    'vendor' : '',
    'part number' : '',
    'cost' : 0.00,
    }


class Camera(object):
    def __init__(self,name='camera1',projection='perspective',angle=65,location=[0,0,100],look_at=[0,0,0],image_size=[640,480],image_dir=''):
        self.info = {}
        self.info['name'] = name
        self.info['projection'] = projection
        self.info['angle'] = angle
        self.info['location'] = location
        self.info['look_at'] = look_at
        self.info['image_size'] = image_size
        self.info['image_name'] = name + '.png'
        self.info['image_dir'] = image_dir
        self.info['image_path'] = os.path.join(self.info['image_dir'],self.info['image_name'])

    def set_parameters(self,obj):
        obj.set_obj_parameter('camera_projection',self.info['projection'])
        obj.set_obj_parameter('camera_angle',self.info['angle'])
        obj.set_obj_parameter('camera_location',self.info['location'])
        obj.set_obj_parameter('camera_look_at',self.info['look_at'])
        obj.set_obj_parameter('image_size',self.info['image_size'])

    def get_info(self):
        return copy.copy(self.info)


class TestRig(csg.Union):
    def __init__(self):
        super(TestRig, self).__init__()
        self.parameters = PARAMETERS
        self.__set_light_sources()
        self.__set_camera_list()
        self.__make_box()
        self.__make_floor_checkerboard()
        self.__make_calibration_checkerboard()
        self.show_box_bool = True
        self.show_floor_bool = False
        self.show_calibration_bool = False
        # self.__set_bom()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def show_box(self,show_box_bool=True):
        self.show_box_bool = bool(show_box_bool)

    def show_floor(self,show_floor_bool=True):
        self.show_floor_bool = bool(show_floor_bool)

    def show_calibration(self,show_calibration_bool=True):
        self.show_calibration_bool = bool(show_calibration_bool)

    def get_camera_info(self,camera_number):
        camera_list_index = camera_number - 1
        camera = self.camera_list[camera_list_index]
        return camera.get_info()

    def render_camera(self,camera_number):
        if self.show_box_bool and (not self.show_calibration_bool):
            self.set_obj_list(self.box)
        else:
            self.set_obj_list([])
        if self.show_floor_bool:
            self.add_obj(self.floor_checkerboard)
        camera_list_index = camera_number - 1
        camera = self.camera_list[camera_list_index]
        camera.set_parameters(self)
        camera_info = camera.get_info()
        if self.show_calibration_bool:
            self.__place_calibration_checkerboard(camera_info)
            self.add_obj(self.calibration_checkerboard)
        self.export(camera_info['image_path'])

    def render_all_cameras(self):
        for camera_list_index in range(len(self.camera_list)):
            camera_number = camera_list_index + 1
            self.render_camera(camera_number)

    def __set_light_sources(self):
        light_source_list = [[100,100,1000],[-100,100,1000],[100,-100,1000],[-100,-100,1000]]
        self.set_obj_parameter('light_source_list',light_source_list)

    def __set_camera_list(self):
        regular_angle = 65
        fisheye_angle = 185
        camera_z = 80
        floor_z = 0
        image_size = [640,480]
        # image_dir = '~/.multi_cam_tracker'
        image_dir = ''
        camera1 = Camera('camera01','perspective',regular_angle,[19,-42.75,camera_z],[19,-42.75,floor_z],image_size,image_dir)
        camera2 = Camera('camera02','perspective',regular_angle,[-19,-42.75,camera_z],[-19,-42.75,floor_z],image_size,image_dir)
        camera3 = Camera('camera03','perspective',regular_angle,[38,-14.25,camera_z],[38,-14.25,floor_z],image_size,image_dir)
        camera4 = Camera('camera04','perspective',regular_angle,[0,-14.25,camera_z],[0,-14.25,floor_z],image_size,image_dir)
        camera5 = Camera('camera05','perspective',regular_angle,[-38,-14.25,camera_z],[-38,-14.25,floor_z],image_size,image_dir)
        camera6 = Camera('camera06','fisheye',fisheye_angle,[0,0,camera_z],[0,0,floor_z],image_size,image_dir)
        camera7 = Camera('camera07','perspective',regular_angle,[38,14.25,camera_z],[38,14.25,floor_z],image_size,image_dir)
        camera8 = Camera('camera08','perspective',regular_angle,[0,14.25,camera_z],[0,14.25,floor_z],image_size,image_dir)
        camera9 = Camera('camera09','perspective',regular_angle,[-38,14.25,camera_z],[-38,14.25,floor_z],image_size,image_dir)
        camera10 = Camera('camera10','perspective',regular_angle,[19,42.75,camera_z],[19,42.75,floor_z],image_size,image_dir)
        camera11 = Camera('camera11','perspective',regular_angle,[-19,42.75,camera_z],[-19,42.75,floor_z],image_size,image_dir)
        self.camera_list = [camera1,camera2,camera3,camera4,camera5,camera6,camera7,camera8,camera9,camera10,camera11]

    def __set_bom(self):
        BOM = bom.BOMObject()
        BOM.set_parameter('name',self.parameters['name'])
        BOM.set_parameter('description',self.parameters['description'])
        BOM.set_parameter('vendor',self.parameters['vendor'])
        BOM.set_parameter('part number',self.parameters['part number'])
        BOM.set_parameter('cost',self.parameters['cost'])
        self.set_obj_parameter('bom',BOM)

    def __make_box(self):
        beam_sl = self.parameters['beam_side_length']
        box_sl = self.parameters['box_side_length']
        beam_length = box_sl - beam_sl
        beam_x = fso.Box(x=beam_length,y=beam_sl,z=beam_sl)
        beams_x = po.LinearArray(beam_x,x=[0],y=[-box_sl/2,box_sl/2],z=0)
        beam_y = fso.Box(x=beam_sl,y=beam_length,z=beam_sl)
        beams_y = po.LinearArray(beam_y,x=[-box_sl/2,box_sl/2],y=0,z=0)
        beam_z = fso.Box(x=beam_sl,y=beam_sl,z=beam_length)
        beams_z = po.LinearArray(beam_z,x=[-box_sl/2,box_sl/2],y=[-box_sl/2,box_sl/2],z=beam_length/2)
        self.box = beams_x | beams_y | beams_z
        self.box.set_color(self.parameters['color'],recursive=True)
        # self.add_obj(self.box)

    def __make_floor_checkerboard(self):
        self.floor_checkerboard = Checkerboard(10,8,8)
        # self.add_obj(self.floor_checkerboard)

    def __make_calibration_checkerboard(self):
        self.calibration_checkerboard = Checkerboard(5,8,6)

    def __place_calibration_checkerboard(self,camera_info):
        projection = camera_info['projection']
        angle = camera_info['angle']
        location = camera_info['location']
        look_at = camera_info['look_at']
        image_size = camera_info['image_size']

        midpoint = [(location[0]+look_at[0])/2,(location[1]+look_at[1])/2,(location[2]+look_at[2])/2]
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
        z_pos = random.randrange((look_at[2]+midpoint[2])/2,(location[2]+midpoint[2])/2)
        self.calibration_checkerboard.set_position([x_pos,y_pos,z_pos])


# ---------------------------------------------------------------------
if __name__ == '__main__':
    test_rig = TestRig()
    test_rig.render_all_cameras()
