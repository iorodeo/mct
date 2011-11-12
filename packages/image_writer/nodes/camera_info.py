#!/usr/bin/env python
"""
Demonstrates converting camera information to ROS image for display with
mjpeg_streamer

Author: Will Dickson
"""
import roslib
roslib.load_manifest('image_writer')
import rospy
import cv 
import Image as PILImage
import ImageDraw as PILImageDraw
import ImageFont as PILImageFont

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge.cv_bridge import CvBridge 
from cv_bridge.cv_bridge import CvBridgeError


class Camera_Info_Image(object):

    def __init__(self):

        self.camera = 'camera1'
        self.topic = '/%s/camera/camera_info'%(self.camera,)
        self.bridge = CvBridge()
        self.font = PILImageFont.truetype("/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-B.ttf", 20)
        self.bg_color = 200
        self.image_size = (320,160) 
        self.text_color = (0,0,200)

        rospy.init_node('image_writer')
        self.pub = rospy.Publisher('image_%s_info'%(self.camera,), Image)
        self.sub = rospy.Subscriber(self.topic,CameraInfo,self.camera_info_handler)

    def run(self):
        rospy.spin()

    def camera_info_handler(self,data):

        # Towards automating this business ...
        attrs = get_user_attributes(data)
        print attrs
        print dir(data.header)
        attrs = get_user_attributes(data.header)
        print attrs
        print 

        camera_text = '%s'%(self.topic,)
        seq_text = '  seq: %d'%(data.header.seq,)
        stamp_text = '  stamp:'
        secs_text = '    secs: %d'%(data.header.stamp.secs,)
        nsecs_text = '    nsecs: %s'%(data.header.stamp.nsecs,)
        text_list = [camera_text, seq_text, stamp_text, secs_text, nsecs_text]

        # Create PIL image, write text to it, and convert to opencv image
        pil_image = PILImage.new('RGB',self.image_size,(self.bg_color,)*3)
        draw = PILImageDraw.Draw(pil_image)

        px, py = 10,10
        for text in text_list:
            draw.text((px,py),text,font=self.font,fill=self.text_color)
            py += 25

        cv_image = cv.CreateImageHeader(pil_image.size, cv.IPL_DEPTH_8U, 3)
        cv.SetData(cv_image, pil_image.tostring())

        # Convert to a rosimage and publish
        rosimage = self.bridge.cv_to_imgmsg(cv_image,'rgb8')
        self.pub.publish(rosimage)


def get_user_attributes(cls,exclude_methods=True):
    base_attrs = dir(type('dummy', (object,), {}))
    this_cls_attrs = dir(cls)
    res = []
    for attr in this_cls_attrs:
        #if base_attrs.count(attr) or (exclude_methods and callable(getattr(cls,attr))):
        #    continue
        if attr[0] == '_' or attr[-1] == '_':
            continue
        res += [attr]
    return res
# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = Camera_Info_Image()
    node.run()



