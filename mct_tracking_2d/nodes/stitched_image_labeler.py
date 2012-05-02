#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_tracking_2d')
import rospy
import sys
import threading
import cv
import cv2
from cv_bridge.cv_bridge import CvBridge 
from mct_utilities import file_tools

# Messages
from sensor_msgs.msg import Image
from mct_msg_and_srv.msg import SeqAndImage
from mct_msg_and_srv.msg import ThreePointTracker


class StitchedImageLabeler(object):

    def __init__(self, stitched_image_topic, tracking_pts_topic, max_seq_age=150):
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.stitching_params = file_tools.read_tracking_2d_stitching_params()
        self.stitched_image_topic = stitched_image_topic
        self.tracking_pts_topic = tracking_pts_topic
        self.max_seq_age = max_seq_age
        self.latest_seq = None
        self.seq_to_stitched_image = {}
        self.seq_to_tracking_pts = {}
        self.cv_text_font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX, 0.8, 0.8,thickness=1)
        self.magenta = (255,255,0)
        self.tracking_pts_colors =[(0,0,255), (0,255,0), (0,255,255)]

        self.ready = False
        rospy.init_node('stitched_image_labeler')

        # Subscribe to stitched image and tracking pts topics
        self.image_sub = rospy.Subscriber(
                self.stitched_image_topic,
                SeqAndImage,
                self.stitched_image_handler
                )
        self.tracking_pts_sub = rospy.Subscriber(
                self.tracking_pts_topic, 
                ThreePointTracker, 
                self.tracking_pts_handler
                )

        # Create labeled image publication
        self.labeled_image_pub = rospy.Publisher('image_stitched_labeled', Image)

        self.ready = True

    def stitched_image_handler(self,data):
        """
        Callback for handling the sequence and stitched image topics.
        """

        if not self.ready:
            return

        with self.lock:
            self.latest_seq = data.seq
            self.seq_to_stitched_image[data.seq] = data.image

    def tracking_pts_handler(self,data):
        """
        Callback for handling the tracking point data from the tracker
        synchronizer.
        """

        if not self.ready:
            return

        with self.lock:
            if data.seq % self.stitching_params['frame_skip'] == 0:
                self.latest_seq = data.seq
                self.seq_to_tracking_pts[data.seq] = data

    def create_labeled_image(self, ros_image, tracking_pts):
        """
        Create labeled version of stitched image using the tracking points data
        """

        # Convert stitched image from rosimage to opencv image. 
        cv_image = self.bridge.imgmsg_to_cv(ros_image, desired_encoding="passthrough")
        ipl_image = cv.GetImage(cv_image)

        # Create color version of stitched image.
        labeled_image = cv.CreateImage(cv.GetSize(ipl_image), cv.IPL_DEPTH_8U,3)
        cv.Zero(labeled_image)
        cv.CvtColor(ipl_image,labeled_image,cv.CV_GRAY2BGR)

        # Write sequence number on image
        message = '{0}'.format(tracking_pts.seq)
        cv.PutText(labeled_image, message, (10,25), self.cv_text_font, self.magenta)
        if tracking_pts.found:
            # Draw boundry box around tracked object
            p_list = [(int(pt.x), int(pt.y)) for pt in tracking_pts.bndry_stitching_plane]
            q_list = p_list[1:] + [p_list[0]]
            for p, q in zip(p_list, q_list):
                cv.Line(labeled_image, p, q, self.magenta)
            # Draw circles around tracked object points
            for pt, color  in zip(tracking_pts.pts_stitching_plane, self.tracking_pts_colors):
                cv.Circle(labeled_image, (int(pt.x), int(pt.y)), 3, color)

        # Convert labeled image to ros image and publish
        labeled_rosimage = self.bridge.cv_to_imgmsg(labeled_image, encoding="passthrough")
        self.labeled_image_pub.publish(labeled_rosimage)


    def run(self):
        """
        Node main loop. Pairs up the tracking points data with the stitched
        images by sequence number and passes them to the create labeled image
        function. Also, cleans out any old data from the seq_to_xxx dictionaries.
        """
        while not rospy.is_shutdown():
            with self.lock:

                # Match up tracking points data with stitched images using sequence number
                for seq, tracking_pts in sorted(self.seq_to_tracking_pts.items()):
                    try:
                        image = self.seq_to_stitched_image[seq]
                        found = True
                    except KeyError:
                        found = False

                    if found:
                        # Reomve items from  storage dictionaries
                        del self.seq_to_stitched_image[seq]
                        del self.seq_to_tracking_pts[seq]
                        self.create_labeled_image(image, tracking_pts)
                    else:
                        # Remove any elements seq_to_tracking_pts dict older than maximum allowed age
                        seq_age = self.latest_seq - seq
                        if seq_age > self.max_seq_age:
                            del self.seq_to_tracking_pts[seq]

                # Remove and elements form seq_to_stitched_image dict older than maximum allowed age
                for seq in self.seq_to_stitched_image.keys():
                    seq_age = self.latest_seq - seq
                    if seq_age > self.max_seq_age:
                        del self.seq_to_stitched_image[seq]
                        

# -----------------------------------------------------------------------------

if __name__ == '__main__':
    stitched_image_topic = sys.argv[1]
    tracking_pts_topic = sys.argv[2]
    node = StitchedImageLabeler(stitched_image_topic, tracking_pts_topic)
    node.run()
