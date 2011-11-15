from __future__ import division
import roslib
roslib.load_manifest('camera_simulation')
import rospy

import cad.finite_solid_objects as fso
import cad.csg_objects as csg
import cad.pattern_objects as po


# Units of inches, scale later to be in mm
PARAMETERS = {
    'color_black' : [0.0,0.0,0.0,1.0],
    'color_white' : [1.0,1.0,1.0,1.0],
    'name' : 'CHECKERBOARD',
    'description' : '',
    'vendor' : '',
    'part number' : '',
    'cost' : 0.00,
    }


class Checkerboard(csg.Union):
    def __init__(self,square_length=1,square_count_x=8,square_count_y=8,thickness=0.01):
        super(Checkerboard, self).__init__()
        self.parameters = PARAMETERS
        self.parameters['square_length'] = square_length

        # Just even for now!
        if square_count_x%2 == 0:
            self.parameters['square_count_x'] = int(square_count_x)
        else:
            self.parameters['square_count_x'] = int(square_count_x) + 1
        if square_count_y%2 == 0:
            self.parameters['square_count_y'] = int(square_count_y)
        else:
            self.parameters['square_count_y'] = int(square_count_y) + 1

        self.parameters['thickness'] = thickness

        self.parameters['length_x'] = self.parameters['square_count_x']*self.parameters['square_length']
        self.parameters['length_y'] = self.parameters['square_count_y']*self.parameters['square_length']

        self.__make_checkerboard()
        # self.__set_bom()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __set_bom(self):
        BOM = bom.BOMObject()
        BOM.set_parameter('name',self.parameters['name'])
        BOM.set_parameter('description',self.parameters['description'])
        BOM.set_parameter('vendor',self.parameters['vendor'])
        BOM.set_parameter('part number',self.parameters['part number'])
        BOM.set_parameter('cost',self.parameters['cost'])
        self.set_object_parameter('bom',BOM)

    def __make_checkerboard(self):
        sl = self.parameters['square_length']
        scx = self.parameters['square_count_x']
        scy = self.parameters['square_count_y']
        t = self.parameters['thickness']

        square = fso.Box(x=sl,y=sl,z=t)
        square_black = square.copy()
        square_black.set_color(self.parameters['color_black'])
        square_white = square.copy()
        square_white.set_color(self.parameters['color_white'])
        offset = sl/2
        checkers_black = po.ArbitraryArray(square_black,[[-offset,-offset,0],[offset,offset,0]])
        checkers_white = po.ArbitraryArray(square_white,[[offset,-offset,0],[-offset,offset,0]])
        checkers = checkers_black | checkers_white

        checkers_count_x = scx//2
        checkers_count_y = scy//2
        checkers_count_x_list = range(checkers_count_x)
        checkers_count_y_list = range(checkers_count_y)
        checkers_end_point_x_negative = -(checkers_count_x - 1)/2
        checkers_end_point_y_negative = -(checkers_count_y - 1)/2
        csl = sl*2
        checkers_x_list = [csl*(checkers_end_point_x_negative + x) for x in checkers_count_x_list]
        checkers_y_list = [csl*(checkers_end_point_y_negative + y) for y in checkers_count_y_list]
        checkerboard = po.LinearArray(checkers,x=checkers_x_list,y=checkers_y_list,z=0)
        self.add_obj(checkerboard)


# ---------------------------------------------------------------------
if __name__ == '__main__':
    checkboard_ = Checkerboard(5,6,8)
    checkboard_.set_object_parameter('camera_projection','fisheye')
    checkboard_.set_object_parameter('camera_angle',185)
    checkboard_.set_object_parameter('camera_location',[0,0,80])
    checkboard_.set_object_parameter('camera_look_at',[0,0,0])
    checkboard_.set_object_parameter('image_size',[640,480])
    checkboard_.export("export.png")
