from __future__ import division
import roslib
roslib.load_manifest('mct_simulation')
import rospy

import cad.finite_solid_objects as fso
import cad.csg_objects as csg
import cad.pattern_objects as po


# Units of inches, scale later to be in mm
PARAMETERS = {
    'color_black' : [0.0,0.0,0.0,1.0],
    'color_white' : [1.0,1.0,1.0,1.0],
    }


class Checkerboard(csg.Union):
    def __init__(self,checker_size=1,checker_count_x=8,checker_count_y=8,thickness=0.001):
        super(Checkerboard, self).__init__()
        self.update_obj_parameters(PARAMETERS)
        self.set_obj_parameter('checker_size',checker_size)

        # Just even for now!
        if checker_count_x%2 == 0:
            self.set_obj_parameter('checker_count_x',int(checker_count_x))
        else:
            self.set_obj_parameter('checker_count_x',int(checker_count_x) + 1)
        if checker_count_y%2 == 0:
            self.set_obj_parameter('checker_count_y',int(checker_count_y))
        else:
            self.set_obj_parameter('checker_count_y',int(checker_count_y) + 1)

        self.set_obj_parameter('checkerboard_thickness',thickness)

        self.set_obj_parameter('checkerboard_length_x',self.get_obj_parameter('checker_count_x')*self.get_obj_parameter('checker_size'))
        self.set_obj_parameter('checkerboard_length_y',self.get_obj_parameter('checker_count_y')*self.get_obj_parameter('checker_size'))

        self.__make_checkerboard()

    def __make_checkerboard(self):
        sl = self.get_obj_parameter('checker_size')
        scx = self.get_obj_parameter('checker_count_x')
        scy = self.get_obj_parameter('checker_count_y')
        t = self.get_obj_parameter('checkerboard_thickness')

        checker = fso.Box(x=sl,y=sl,z=t)
        checker_black = checker.copy()
        checker_black.set_color(self.get_obj_parameter('color_black'))
        checker_white = checker.copy()
        checker_white.set_color(self.get_obj_parameter('color_white'))
        offset = sl/2
        checkers_black = po.ArbitraryArray(checker_black,[[-offset,-offset,0],[offset,offset,0]])
        checkers_white = po.ArbitraryArray(checker_white,[[offset,-offset,0],[-offset,offset,0]])
        # checkers = checkers_black | checkers_white
        checkers = checkers_black

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
    checkerboard_ = Checkerboard(5,6,8)
    checkerboard_.add_camera('checkerboard','fisheye',185,[0,0,80],[0,0,0],[640,480])
    checkerboard_.render()
