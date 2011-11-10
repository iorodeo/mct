from __future__ import division
import roslib
roslib.load_manifest('camera_simulation')
import rospy

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
    def __init__(self,name='camera1',projection='perspective',angle=65,location=[0,0,100],look_at=[0,0,0],image_size=[640,480]):
        self.name = name
        self.projection = projection
        self.angle = angle
        self.location = location
        self.look_at = look_at
        self.image_size = image_size

    def set_parameters(self,obj):
        obj.set_object_parameter('camera_projection',self.projection)
        obj.set_object_parameter('camera_angle',self.angle)
        obj.set_object_parameter('camera_location',self.location)
        obj.set_object_parameter('camera_look_at',self.look_at)
        obj.set_object_parameter('image_size',self.image_size)

    def get_image_name(self):
        return self.name + '.png'


class TestRig(csg.Union):
    def __init__(self):
        super(TestRig, self).__init__()
        self.parameters = PARAMETERS
        self.__set_light_sources()
        self.__set_camera_list()
        self.__make_box()
        self.__make_checkerboard_floor()
        # self.__set_bom()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def render_camera(self,camera_number):
        camera_list_index = camera_number - 1
        camera = self.camera_list[camera_list_index]
        camera.set_parameters(self)
        self.export(camera.get_image_name())

    def render_all_cameras(self):
        for camera_list_index in range(len(self.camera_list)):
            camera_number = camera_list_index + 1
            self.render_camera(camera_number)

    def __set_light_sources(self):
        light_source_list = [[100,100,1000],[-100,100,1000],[100,-100,1000],[-100,-100,1000]]
        self.set_object_parameter('light_source_list',light_source_list)

    def __set_camera_list(self):
        regular_angle = 65
        fisheye_angle = 185
        camera_z = 80
        floor_z = 0
        image_size = [640,480]
        camera1 = Camera('camera01','perspective',regular_angle,[19,-42.75,camera_z],[19,-42.75,floor_z],image_size)
        camera2 = Camera('camera02','perspective',regular_angle,[-19,-42.75,camera_z],[-19,-42.75,floor_z],image_size)
        camera3 = Camera('camera03','perspective',regular_angle,[38,-14.25,camera_z],[38,-14.25,floor_z],image_size)
        camera4 = Camera('camera04','perspective',regular_angle,[0,-14.25,camera_z],[0,-14.25,floor_z],image_size)
        camera5 = Camera('camera05','perspective',regular_angle,[-38,-14.25,camera_z],[-38,-14.25,floor_z],image_size)
        camera6 = Camera('camera06','fisheye',fisheye_angle,[0,0,camera_z],[0,0,floor_z],image_size)
        camera7 = Camera('camera07','perspective',regular_angle,[38,14.25,camera_z],[38,14.25,floor_z],image_size)
        camera8 = Camera('camera08','perspective',regular_angle,[0,14.25,camera_z],[0,14.25,floor_z],image_size)
        camera9 = Camera('camera09','perspective',regular_angle,[-38,14.25,camera_z],[-38,14.25,floor_z],image_size)
        camera10 = Camera('camera10','perspective',regular_angle,[19,42.75,camera_z],[19,42.75,floor_z],image_size)
        camera11 = Camera('camera11','perspective',regular_angle,[-19,42.75,camera_z],[-19,42.75,floor_z],image_size)
        self.camera_list = [camera1,camera2,camera3,camera4,camera5,camera6,camera7,camera8,camera9,camera10,camera11]

    def __set_bom(self):
        BOM = bom.BOMObject()
        BOM.set_parameter('name',self.parameters['name'])
        BOM.set_parameter('description',self.parameters['description'])
        BOM.set_parameter('vendor',self.parameters['vendor'])
        BOM.set_parameter('part number',self.parameters['part number'])
        BOM.set_parameter('cost',self.parameters['cost'])
        self.set_object_parameter('bom',BOM)

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
        box = beams_x | beams_y | beams_z
        box.set_color(self.parameters['color'],recursive=True)
        self.add_obj(beams_x)
        self.add_obj(beams_y)
        self.add_obj(beams_z)

    def __make_checkerboard_floor(self):
        cb = Checkerboard(10,8,8)
        self.add_obj(cb)


# ---------------------------------------------------------------------
if __name__ == '__main__':
    test_rig = TestRig()
    test_rig.render_all_cameras()
