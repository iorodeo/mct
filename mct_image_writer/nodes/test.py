#!/usr/bin/env python
"""
Examples demonstrating how to publish various types of data (matplotlib
figures, numpy arrays, opencv image) as ros images. These can then be viewed
from a web browser using mjpeg_streamer.

Author: Will Dickson
"""
import roslib
roslib.load_manifest('mct_iamge_writer')
import rospy
import numpy
import cv
from matplotlib import pyplot
import pylab
import Image as PILImage
import ImageDraw as PILImageDraw
import ImageFont as PILImageFont

from sensor_msgs.msg import Image
from cv_bridge.cv_bridge import CvBridge
from cv_bridge.cv_bridge import CvBridgeError


class ImWriter_PyPlot_Figure(object):
    """
    Demonstrates how to publish an animated matplotlib figure as a ROS image.
    """
    def __init__(self):
        rospy.init_node('mct_iamge_writer')
        self.pub = rospy.Publisher('image_test', Image)
        self.rate = rospy.Rate(20)
        self.bridge = CvBridge()
        self.count = 0

        self.fig = pyplot.figure()
        self.plot = self.fig.add_subplot(111)
        self.plot.set_title('sinwave')
        self.axes = self.plot.plot([], [])[0]
        self.plot.set_xlim(0,30)
        self.plot.set_ylim(-2,2)
        self.plot.set_ylabel('x (m)')
        self.plot.set_xlabel('t (sec)')
        self.plot.grid('on')
        a = fig2data(self.fig)
        self.t_list = []
        self.x_list = []
        self.t_start = rospy.get_time()
        self.t_offset = 0

    def run(self):

        while not rospy.is_shutdown():

            # Get next time and data point and adjust for time offset.
            t_next = rospy.get_time() - self.t_start
            x_next = numpy.sin(t_next)
            t_next += self.t_offset
            self.t_list.append(t_next)
            self.x_list.append(x_next)

            t_max = self.plot.get_xlim()[1]
            if t_next > t_max:
                dt = t_next - t_max
                self.t_offset -= dt
                combined_list = zip(self.t_list,self.x_list)
                combined_list = [(t-dt,x)  for (t,x) in combined_list if (t-dt) >= 0]
                self.t_list = [t for (t,x) in combined_list]
                self.x_list = [x for (t,x) in combined_list]

            # Update figure data
            self.axes.set_data(self.t_list,self.x_list)

            # Convert to opencv image
            data = fig2data(self.fig)
            cv_image = cv.CreateImageHeader((data.shape[0], data.shape[1]), cv.IPL_DEPTH_8U, 4)
            cv.SetData(cv_image,data)

            # Convert to ROS image
            rosimage = self.bridge.cv_to_imgmsg(cv_image,'rgba8')
            self.pub.publish(rosimage)
            self.rate.sleep()
            self.count += 1

class ImWriter_OpenCv(object):
    """
    Demonstrates how to publish opencv images as ros images
    """

    def __init__(self):
        rospy.init_node('mct_iamge_writer')
        self.pub = rospy.Publisher('image_test', Image)
        self.rate = rospy.Rate(20)
        self.bridge = CvBridge()
        self.count = 0

        self.cv_text_font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX, 0.8, 0.8,thickness=1)
        self.cv_text_color = cv.Scalar(0,255,0)

    def run(self):

        while not rospy.is_shutdown():
            image_text = 'count: %d'%(self.count,)
            cv_image = cv.CreateImage((800,400), cv.IPL_DEPTH_8U,3)
            color = cv.Scalar(10,10,10)
            cv.Set(cv_image,color)
            for i in range(0,8):
                for j in range(0,4):
                    cv.PutText(cv_image,image_text,(20+200*j,50 + i*40),self.cv_text_font,self.cv_text_color)

            rosimage = self.bridge.cv_to_imgmsg(cv_image,'bgr8')
            self.pub.publish(rosimage)
            self.rate.sleep()
            self.count += 1

class ImWriter_Numpy(object):
    """
    Demonstrates how to publish a numpy array as a ROS image.
    """
    def __init__(self):
        rospy.init_node('mct_iamge_writer')
        self.pub = rospy.Publisher('image_test', Image)
        self.rate = rospy.Rate(20)
        self.bridge = CvBridge()
        self.count_red = 0
        self.count_green = 0
        self.count_blue = 0

    def run(self):

        while not rospy.is_shutdown():


            # Creating numpy array and convert it to an opencv image
            a = numpy.zeros((300,300,3),dtype='uint8')
            a[50:100,25:75,0] = self.count_red%256
            a[100:150,75:125,1] = self.count_green%256
            a[150:200,125:175,2] = self.count_blue%256
            cv_image = cv.fromarray(a)

            # Convert to a rosimage and publish
            rosimage = self.bridge.cv_to_imgmsg(cv_image,'rgb8')
            self.pub.publish(rosimage)
            self.rate.sleep()
            self.count_red += 1
            self.count_green += 2
            self.count_blue += 3


class ImWriter_PIL(object):
    """
    Demonstrates how to publish a PIL image as a ROS image.
    """
    def __init__(self):
        rospy.init_node('mct_iamge_writer')
        self.pub = rospy.Publisher('image_test', Image)
        self.rate = rospy.Rate(20)
        self.bridge = CvBridge()
        self.count = 0
        self.font = PILImageFont.truetype("/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-B.ttf", 20)

    def run(self):

        while not rospy.is_shutdown():

            # Create PIL image, write text to it, and convert to opencv image
            pil_image = PILImage.new('RGB',(180,50),(255,255,255))
            draw = PILImageDraw.Draw(pil_image)
            image_text = 'count: %d'%(self.count,)
            draw.text((10,10),image_text,font=self.font,fill=(0,0,0))
            cv_image = cv.CreateImageHeader(pil_image.size, cv.IPL_DEPTH_8U, 3)
            cv.SetData(cv_image, pil_image.tostring())

            # Convert to a rosimage and publish
            rosimage = self.bridge.cv_to_imgmsg(cv_image,'rgb8')
            self.pub.publish(rosimage)
            self.rate.sleep()
            self.count += 1

# Utility functions
# -----------------------------------------------------------------------------
def fig2data(fig):
    """
        Convert a Matplotlib figure to a 4D numpy array with RGBA channels and
        return it fig a matplotlib figure a numpy 3D array of RGBA values
    """
    # draw the renderer
    fig.canvas.draw ( )
    # Get the RGBA buffer from the figure
    w,h = fig.canvas.get_width_height()
    buf = numpy.fromstring ( fig.canvas.tostring_argb(), dtype=numpy.uint8 )
    buf.shape = (w, h,4)
    # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to have it in RGBA mode
    buf = numpy.roll ( buf, 3, axis = 2 )
    return buf

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    if 0:
        node = ImWriter_PyPlot_Figure()
    if 1:
        node = ImWriter_PIL()
    if 0:
        node = ImWriter_OpenCv()
    if 0:
        node = ImWriter_Numpy()

    node.run()

