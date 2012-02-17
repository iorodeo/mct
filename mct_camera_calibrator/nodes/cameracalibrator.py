#!/usr/bin/python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# WBD. 
# ---------------------------------------------------------------------
#
# In order to support the functionaliry required by the MCT application 
# this nodes has been modified form the original version developed by 
# Willow Garage. In general the OpenCV based user interface has been 
# removed. Instead calibration progress images are published - which 
# meant to be viewed as mjpeg stream from a web browser. In addition
# 'calibrate' and get calibration services have been added.
# ---------------------------------------------------------------------

PKG = 'mct_camera_calibrator' # this package name
import roslib; roslib.load_manifest(PKG)

import rospy
import sensor_msgs.msg
import sensor_msgs.srv
import message_filters

import os
import Queue
import threading
import functools

import cv
import cv2
from cv_bridge.cv_bridge import CvBridge

from mct_camera_calibrator.approxsync import ApproximateSynchronizer
from mct_camera_calibrator.calibrator import cvmat_iterator 
from mct_camera_calibrator.calibrator import MonoCalibrator 
from mct_camera_calibrator.calibrator import StereoCalibrator 
from mct_camera_calibrator.calibrator import ChessboardInfo

# Messages and Services
import sensor_msgs.msg
import sensor_msgs.srv
from mct_msg_and_srv.srv import GetBool
from mct_msg_and_srv.srv import GetBoolResponse
#from mct_msg_and_srv.srv import CommandString
#from mct_msg_and_srv.srv import CommandStringResponse
from mct_msg_and_srv.srv import GetString
from mct_msg_and_srv.srv import GetStringResponse

class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function

    def run(self):
        while True:
            while True:
                m = self.queue.get()
                if self.queue.empty():
                    break
            self.function(m)


class CalibrationNode(object):
    def __init__(self, boards, service_check = True, synchronizer = message_filters.TimeSynchronizer, flags = 0):
        if service_check:
            # assume any non-default service names have been set.  Wait for the service to become ready
            for svcname in ["camera", "left_camera", "right_camera"]:
                remapped = rospy.remap_name(svcname)
                if remapped != svcname:
                    fullservicename = "%s/set_camera_info" % remapped
                    print "Waiting for service", fullservicename, "..."
                    try:
                        rospy.wait_for_service(fullservicename, 5)
                        print "OK"
                    except rospy.ROSException:
                        print "Service not found"
                        rospy.signal_shutdown('Quit')

        self._boards = boards
        self._calib_flags = flags
        lsub = message_filters.Subscriber('left', sensor_msgs.msg.Image)
        rsub = message_filters.Subscriber('right', sensor_msgs.msg.Image)
        ts = synchronizer([lsub, rsub], 4)
        ts.registerCallback(self.queue_stereo)

        msub = message_filters.Subscriber('image', sensor_msgs.msg.Image)
        msub.registerCallback(self.queue_monocular)
        
        self.set_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("camera"),
                                                          sensor_msgs.srv.SetCameraInfo)
        self.set_left_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("left_camera"),
                                                               sensor_msgs.srv.SetCameraInfo)
        self.set_right_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("right_camera"),
                                                                sensor_msgs.srv.SetCameraInfo)

        self.q_mono = Queue.Queue()
        self.q_stereo = Queue.Queue()

        self.c = None

        mth = ConsumerThread(self.q_mono, self.handle_monocular)
        mth.setDaemon(True)
        mth.start()

        sth = ConsumerThread(self.q_stereo, self.handle_stereo)
        sth.setDaemon(True)
        sth.start()
        
    def redraw_stereo(self, *args):
        pass
    def redraw_monocular(self, *args):
        pass

    def queue_monocular(self, msg):
        self.q_mono.put(msg)

    def queue_stereo(self, lmsg, rmsg):
        self.q_stereo.put((lmsg, rmsg))

    def handle_monocular(self, msg):
        if self.c == None:
            self.c = MonoCalibrator(self._boards, self._calib_flags)

        # This should just call the MonoCalibrator
        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.scrib.cols
        self.redraw_monocular(drawable)

    def handle_stereo(self, msg):
        if self.c == None:
            self.c = StereoCalibrator(self._boards, self._calib_flags)
            
        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.lscrib.cols + drawable.rscrib.cols
        self.redraw_stereo(drawable)
            
 
    def check_set_camera_info(self, response):
        if response.success:
            return True

        for i in range(10):
            print "!" * 80
        print
        print "Attempt to set camera info failed: " + response.status_message
        print
        for i in range(10):
            print "!" * 80
        print
        rospy.logerr('Unable to set camera info for calibration. Failure message: %s' % response.status_message)
        return False

    def do_upload(self):
        self.c.report()
        print self.c.ost()
        info = self.c.as_message()

        rv = True
        if self.c.is_mono:
            response = self.set_camera_info_service(info)
            rv = self.check_set_camera_info(response)
        else:
            response = self.set_left_camera_info_service(info[0])
            rv = rv and self.check_set_camera_info(response)
            response = self.set_right_camera_info_service(info[1])
            rv = rv and self.check_set_camera_info(response)
        return rv

# WBD
# -----------------------------------------------------------------------------
class MCT_CalibrationNode(CalibrationNode):

    def __init__(self,*args,**kwargs):
        self.lock = threading.Lock()
        self.calibrate = False
        super(MCT_CalibrationNode,self).__init__(*args,**kwargs)
        self.bridge = CvBridge()
        self.cal_img_pub = rospy.Publisher('image_calibrator', sensor_msgs.msg.Image)

        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX,1,1,0,2)
        self.font_color_red = cv.CV_RGB(255,0,0)
        self.font_color_green = cv.CV_RGB(0,255,0)
        self.font_color_blue = cv.CV_RGB(80,80,255)
        self.x_start = 10
        self.y_start = 30
        self.x_step = 140
        self.y_step = 32

        # Set up sevices
        node_name= rospy.get_name()
        self.good_enough_srv = rospy.Service(
                '{0}/good_enough'.format(node_name),
                GetBool,
                self.handle_good_enough_srv,
                )

        self.calibrate_srv = rospy.Service(
                '{0}/calibrate'.format(node_name),
                GetBool,
                self.handle_calibrate_srv,
                )

        self.get_calibration_srv = rospy.Service(
                '{0}/get_calibration'.format(node_name),
                GetString,
                self.handle_get_calibration_srv,
                )

    def handle_good_enough_srv(self,req):
        """
        Handles requests for the calibrators good_enough flag which indicates whether
        or not sufficient data has been aquired for calibrating the camera.
        """
        with self.lock:
            good_enough = self.c.goodenough
        return GetBoolResponse(good_enough)

    def handle_calibrate_srv(self,req):
        """
        Handles requests for calibrating the cameras associated with the calibrator.
        """
        with self.lock:
            if self.c.goodenough:
                self.calibrate = True
                flag = True
            else:
                flag = False
        return GetBoolResponse(flag) 

    def handle_get_calibration_srv(self,req):
        """
        Handles requests for the camera calibration parameters.
        """
        with self.lock:
            if self.c.calibrated:
                ost_txt = self.c.ost()
            else:
                ost_txt = ''

        if ost_txt:
            # Add actuall camera name to calibration
            camera_topic = rospy.remap_name('camera')
            camera_name= camera_topic.split('/')[2]
            ost_txt = ost_txt.replace('narrow_stereo/left',camera_name)
            
        return GetStringResponse(ost_txt)

    def redraw_monocular(self, drawable):
        """
        Redraw calibratoin image callback.
        """
        if not self.c.calibrated: 

            with self.lock:
                if self.calibrate and self.c.goodenough:
                    self.c.do_calibration()

            if self.c.goodenough:
                text_data = [('Qty', '{0}'.format(len(self.c.db))),  ('Good Enough',)]
                self.add_progress_text(drawable.scrib,text_data,self.font_color_green)
            else:
                if drawable.params:
                    text_data =  [('Qty', '{0}'.format(len(self.c.db)) )] + list(drawable.params)
                    self.add_progress_text(drawable.scrib,text_data,self.font_color_red)
                else:
                    text_data = [('No Data',)]
                    self.add_progress_text(drawable.scrib,text_data,self.font_color_red)

        else:
            text_data = [('Calibrated',)]
            self.add_progress_text(drawable.scrib,text_data,self.font_color_green)
            #print self.c.ost()

        rosimage = self.bridge.cv_to_imgmsg(drawable.scrib,'bgr8')
        self.cal_img_pub.publish(rosimage)

    def add_progress_text(self, img, text_data, font_color): 
        """
        Adds progress text to calibration image, img. The text to be added
        consists of a list of tuples where each tuple is a row of text to be
        added.
        """
        for i, values in enumerate(text_data): 
            for j, item in enumerate(values): 
                if type(item) == float: 
                    msg = '{0:1.2f}'.format(item) 
                else: 
                    msg = '{0}'.format(item) 
                pos = (self.x_start + j*self.x_step, self.y_start + i*self.y_step)
                cv.PutText(img, msg, pos, self.font, font_color)


def main():
    from optparse import OptionParser, OptionGroup
    parser = OptionParser("%prog --size SIZE1 --square SQUARE1 [ --size SIZE2 --square SQUARE2 ]",
                          description=None)
    group = OptionGroup(parser, "Chessboard Options",
                        "You must specify one or more chessboards as pairs of --size and --square options.")
    #group.add_option("--pattern",
    #                 type="string", default="chessboard",
    #                 help="calibration pattern to detect - 'chessboard' or 'circles'")
    group.add_option("-s", "--size",
                     action="append", default=[],
                     help="chessboard size as NxM, counting interior corners (e.g. a standard chessboard is 7x7)")
    group.add_option("-q", "--square",
                     action="append", default=[],
                     help="chessboard square size in meters")
    parser.add_option_group(group)
    group = OptionGroup(parser, "ROS Communication Options")
    group.add_option("--approximate",
                     type="float", default=0.0,
                     help="allow specified slop (in seconds) when pairing images from unsynchronized stereo cameras")
    group.add_option("--no-service-check",
                     action="store_false", dest="service_check", default=True,
                     help="disable check for set_camera_info services at startup")
    parser.add_option_group(group)
    group = OptionGroup(parser, "Calibration Optimizer Options")
    group.add_option("--fix-principal-point",
                     action="store_true", default=False,
                     help="fix the principal point at the image center")
    group.add_option("--fix-aspect-ratio",
                     action="store_true", default=False,
                     help="enforce focal lengths (fx, fy) are equal")
    group.add_option("--zero-tangent-dist",
                     action="store_true", default=False,
                     help="set tangential distortion coefficients (p1, p2) to zero")
    group.add_option("-k", "--k-coefficients",
                     type="int", default=2, metavar="NUM_COEFFS",
                     help="number of radial distortion coefficients to use (up to 6, default %default)")
    parser.add_option_group(group)
    group = OptionGroup(parser, "Deprecated Options")
    group.add_option("--rational-model",
                     action="store_true", default=False,
                     help="enable distortion coefficients k4, k5 and k6 (for high-distortion lenses)")
    group.add_option("--fix-k1", action="store_true", default=False,
                     help="do not change the corresponding radial distortion coefficient during the optimization")
    group.add_option("--fix-k2", action="store_true", default=False)
    group.add_option("--fix-k3", action="store_true", default=False)
    group.add_option("--fix-k4", action="store_true", default=False)
    group.add_option("--fix-k5", action="store_true", default=False)
    group.add_option("--fix-k6", action="store_true", default=False)
    parser.add_option_group(group)
    options, args = parser.parse_args()

    if len(options.size) != len(options.square):
        parser.error("Number of size and square inputs must be the same!")
    
    if not options.square:
        options.square.append("0.108")
        options.size.append("8x6")

    boards = []
    for (sz, sq) in zip(options.size, options.square):
        size = tuple([int(c) for c in sz.split('x')])
        boards.append(ChessboardInfo(size[0], size[1], float(sq)))

    if options.approximate == 0.0:
        sync = message_filters.TimeSynchronizer
    else:
        sync = functools.partial(ApproximateSynchronizer, options.approximate)

    num_ks = options.k_coefficients
    # Deprecated flags modify k_coefficients
    if options.rational_model:
        print "Option --rational-model is deprecated"
        num_ks = 6
    if options.fix_k6:
        print "Option --fix-k6 is deprecated"
        num_ks = min(num_ks, 5)
    if options.fix_k5:
        print "Option --fix-k5 is deprecated"
        num_ks = min(num_ks, 4)
    if options.fix_k4:
        print "Option --fix-k4 is deprecated"
        num_ks = min(num_ks, 3)
    if options.fix_k3:
        print "Option --fix-k3 is deprecated"
        num_ks = min(num_ks, 2)
    if options.fix_k2:
        print "Option --fix-k2 is deprecated"
        num_ks = min(num_ks, 1)
    if options.fix_k1:
        print "Option --fix-k1 is deprecated"
        num_ks = 0

    calib_flags = 0
    if options.fix_principal_point:
        calib_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    if options.fix_aspect_ratio:
        calib_flags |= cv2.CALIB_FIX_ASPECT_RATIO
    if options.zero_tangent_dist:
        calib_flags |= cv2.CALIB_ZERO_TANGENT_DIST
    if (num_ks > 3):
        calib_flags |= cv2.CALIB_RATIONAL_MODEL
    if (num_ks < 6):
        calib_flags |= cv2.CALIB_FIX_K6
    if (num_ks < 5):
        calib_flags |= cv2.CALIB_FIX_K5
    if (num_ks < 4):
        calib_flags |= cv2.CALIB_FIX_K4
    if (num_ks < 3):
        calib_flags |= cv2.CALIB_FIX_K3
    if (num_ks < 2):
        calib_flags |= cv2.CALIB_FIX_K2
    if (num_ks < 1):
        calib_flags |= cv2.CALIB_FIX_K1

    rospy.init_node('cameracalibrator')
    node = MCT_CalibrationNode(boards, options.service_check, sync, calib_flags)
    rospy.spin()

# -----------------------------------------------------------------------------
if __name__ == "__main__":
    try:
        main()
    except Exception, e:
        import traceback
        traceback.print_exc()
