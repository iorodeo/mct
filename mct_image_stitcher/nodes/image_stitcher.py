#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_image_stitcher')
import rospy
import cv
import cv2
import sys
import threading
import functools
import math
import numpy

import mct_introspection
from mct_utilities import file_tools
from mct_transform_2d import transform_2d
from cv_bridge.cv_bridge import CvBridge 

# Messages
from mct_msg_and_srv.msg import StampInfo
from mct_msg_and_srv.msg import ProcessingInfo
from mct_msg_and_srv.msg import SeqAndImage
from sensor_msgs.msg import CameraInfo 
from sensor_msgs.msg import Image 
from std_msgs.msg import UInt32
from std_msgs.msg import Float32

# Services
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

MAX_UINT32 = 2**32 - 1

class ImageStitcher(object):
    """
    Subscribes to all rectified image topics in a 2d tracking region and uses
    the calibration data to stitch the images together. In turn it publishes
    image_stitched and image_stitched/seq.

    Note, in order to handle drop frames gracefully I've modified this node
    so that it can subscribe to the mct_msg_and_srv/SeqAndImage topics published
    by the frame_drop_corrector nodes. 
    """

    def __init__(
            self, 
            region, 
            topic_end='image_rect_skip', 
            topic_type='sensor_msgs/Image', 
            max_seq_age=150, 
            max_stamp_age=1.5
            ):

        self.topic_end = topic_end
        self.topic_type = topic_type
        self.max_seq_age = max_seq_age
        self.max_stamp_age = max_stamp_age

        self.warp_flags = cv.CV_INTER_LINEAR 
        self.stitching_params = file_tools.read_tracking_2d_stitching_params()

        rospy.init_node('image_stitcher')
        self.ready = False
        self.is_first_write = True
        self.stitched_image = None
        self.seq_to_images = {}  
        self.stamp_to_seq_pool= {}
        self.image_waiting_pool = {} 

        # Information on the latest sequence received and stitched
        self.seq_newest = None
        self.stamp_newest = None

        self.lock = threading.Lock()
        self.bridge = CvBridge()

        # Get image and camera information 
        regions_dict = file_tools.read_tracking_2d_regions()
        self.region = region
        self.camera_list = regions_dict[region]
        self.create_camera_to_image_dict()
        self.create_camera_to_info_dict()

        # Get transforms from cameras to stitched image and stitched image size
        self.tf2d = transform_2d.Transform2d()
        self.create_tf_data()
        self.get_stitched_image_size()

        # Subscribe to topics based on incoming topic type.
        if self.topic_type == 'sensor_msgs/Image':
            
            # Create pool dictionaries for incomming data.  
            for camera in self.camera_list:
                self.stamp_to_seq_pool[camera] = {}
                self.image_waiting_pool[camera] = {}

            # Subscribe to camera info topics
            self.info_sub = {} 
            for camera, topic  in  self.camera_to_info.iteritems():
                info_handler = functools.partial(self.info_handler, camera)
                self.info_sub[camera] = rospy.Subscriber(topic, CameraInfo, info_handler)

            # Subscribe to rectified image topics
            self.image_sub = {} 
            for camera, topic  in  self.camera_to_image.iteritems():
                image_handler = functools.partial(self.image_handler, camera)
                self.image_sub[camera] = rospy.Subscriber(topic, Image, image_handler)

        elif self.topic_type == 'mct_msg_and_srv/SeqAndImage':

            # Subscribe to SeqAndImage topics
            self.seq_and_image_sub = {}
            for camera, topic in self.camera_to_image.iteritems():
                seq_and_image_handler = functools.partial(self.seq_and_image_handler, camera)
                self.seq_and_image_sub[camera] = rospy.Subscriber(topic, SeqAndImage, seq_and_image_handler)

        else:
            err_msg = 'unable to handle topic type: {0}'.format(self.topic_type)
            raise ValueError, err_msg

        # Stitched image publisher and seq publisher
        self.image_pub = rospy.Publisher('image_stitched', Image)
        self.image_and_seq_pub = rospy.Publisher('seq_and_image_stitched', SeqAndImage)
        self.seq_pub = rospy.Publisher('image_stitched/seq', UInt32)
        self.stamp_pub = rospy.Publisher('image_stitched/stamp_info', StampInfo)
        self.processing_dt_pub = rospy.Publisher('image_stitched/processing_dt', ProcessingInfo)

        # Setup reset service - needs to be called anytime the camera trigger is  
        # stopped - before it is restarted. Empties buffers of images and sequences. 
        self.reset_srv = rospy.Service('reset_image_stitcher', Empty, self.handle_reset_srv)
        self.ready = True

    def handle_reset_srv(self, req):
        """
        Handles the nodes reset service - which empties the image and sequence buffers.
        """
        with self.lock:
            self.seq_to_images = {}  
            self.stamp_to_seq_pool= {}
            self.image_waiting_pool = {} 
            self.seq_newest = None
            self.stamp_newest = None
        return EmptyResponse()

    def create_camera_to_image_dict(self):
        """
        Create camera to image topic dictionary
        """
        self.camera_to_image = {}
        image_topics = mct_introspection.find_camera_image_topics(transport=self.topic_end)
        #for topic in rect_topics:
        for topic in image_topics:
            topic_split = topic.split('/')
            for camera in self.camera_list:
                if camera in topic_split:
                    self.camera_to_image[camera] = topic

    def create_camera_to_info_dict(self):
        """
        Create camera to image topic dictionary
        """
        self.camera_to_info = {}
        info_topics = mct_introspection.find_camera_info_topics()
        for topic in info_topics:
            topic_split = topic.split('/')
            for camera in self.camera_list:
                if camera in topic_split:
                    self.camera_to_info[camera] = topic

    def create_tf_data(self):
        """
        Pre-computes transform matrices and roi for creating the stitched
        images.
        """
        self.tf_data = {}
        for camera in self.camera_list:

            # Get modified transform which maps to ROI in stitched image 
            bbox = self.tf2d.get_stitching_plane_bounding_box(
                    region, 
                    camera_list=[camera]
                    )
            tf_matrix = self.tf2d.get_camera_to_stitching_plane_tf(camera)
            roi_x = int(math.floor(bbox['min_x']))
            roi_y = int(math.floor(bbox['min_y']))
            roi_w = int(math.floor(bbox['max_x']) - math.floor(bbox['min_x']+5))
            roi_h = int(math.floor(bbox['max_y']) - math.floor(bbox['min_y']+5))
            shift_matrix = numpy.array([
                [1.0, 0.0, -bbox['min_x']],
                [0.0, 1.0, -bbox['min_y']], 
                [0.0, 0.0, 1.0],
                ])
            tf_matrix = numpy.dot(shift_matrix, tf_matrix)
            self.tf_data[camera] = {
                    'matrix': cv.fromarray(tf_matrix), 
                    'roi': (roi_x, roi_y, roi_w, roi_h)
                    }

    def get_stitched_image_size(self):
        """
        Determines the size of the stitched image.
        """
        # Get stitched image size from bounding box
        bbox = self.tf2d.get_stitching_plane_bounding_box(region)
        self.image_width = int(math.ceil(bbox['max_x']))
        self.image_height = int(math.ceil(bbox['max_y']))
        self.image_size = (self.image_width, self.image_height)

    def info_handler(self,camera,data):
        """
        Handler for incoming camera info messages. In this callback we place
        image sequence number into stamp_to_seq_pool dictionay by camera name
        and timestamp 

        Note, only used when imcoming topics are of type sensor_msgs/Image. In
        this case both the Image and camera_info topics are need to associate
        the acquisition sequence numbers with the images as the seq numbers in
        the image headers aren't reliable.
        """
        if self.ready:
            with self.lock:
                stamp = data.header.stamp.secs, data.header.stamp.nsecs
                self.stamp_to_seq_pool[camera][stamp] = data.header.seq
                self.update_seq_newest(data.header.seq)
                self.update_stamp_newest(stamp)


    def image_handler(self, camera, data):
        """
        Handler for incoming camera images. In this callback we place the image
        data into the image_waiting_pool by camera name and timestamp. Note, we
        don't want to use the seq information in data.header.seq as this seems
        to initialized in some random way - probably has to do with python not
        fully supporting ROS's image_transport. The seq's from the camera_info
        topics are correct.

        Note, only used when imcoming topics are of type sensor_msgs/Image. In
        this case both the Image and camera_info topics are need to associate
        the acquisition sequence numbers with the images as the seq numbers in
        the image headers aren't reliable.
        """
        if self.ready:
            with self.lock:
                stamp = data.header.stamp.secs, data.header.stamp.nsecs
                self.image_waiting_pool[camera][stamp] = data

    def seq_and_image_handler(self,camera,data):
        """
        Handler for incoming SeqAndImage messages which contain both the image
        data and the frame drop corrected acquisition sequence numbers.
        
        Note, onle used when the incomind topics are of type
        mct_msg_and_srv/SeqAndImage.
        """
        if self.ready:
            with self.lock:
                try:
                    self.seq_to_images[data.seq][camera] = data.image
                except KeyError:
                    self.seq_to_images[data.seq] = {camera: data.image}
                self.update_seq_newest(data.seq)

    def update_seq_newest(self,seq): 
        if self.seq_newest is None:
            self.seq_newest = seq
        else:
            self.seq_newest = max([self.seq_newest, seq])

    def update_stamp_newest(self,stamp): 
        if self.stamp_newest is None:
            self.stamp_newest = stamp
        else:
            self.stamp_newest = max([self.stamp_newest, stamp])

    def process_waiting_images(self):
        """ 
        Processes waiting images. Associates images in the waiting pool with
        their acquisition sequence number and places them in the seq_to_images
        buffer.         
        """
        with self.lock:
            # Associate images in the waiting pool with their seq numbers 
            for camera in self.camera_list:
                for stamp, image_data in self.image_waiting_pool[camera].items():

                    try:
                        seq = self.stamp_to_seq_pool[camera][stamp]
                        del self.stamp_to_seq_pool[camera][stamp]
                        del self.image_waiting_pool[camera][stamp]
                        image_data.header.seq = seq 
                        deleted = True
                        try:
                            self.seq_to_images[seq][camera] = image_data
                        except KeyError:
                            self.seq_to_images[seq] = {camera: image_data}
                    except KeyError:
                        deleted = False

                    # Clear out stale data from image waiting pool
                    if not deleted:
                        stamp_age = self.get_stamp_age(stamp)
                        if stamp_age > self.max_stamp_age:
                            del self.image_waiting_pool[camera][stamp]

                # Clear out any stale data from stamp-to-seq pool
                for stamp, seq in self.stamp_to_seq_pool[camera].items():
                    seq_age = self.get_seq_age(seq)
                    if seq_age > self.max_seq_age: 
                        del self.stamp_to_seq_pool[camera][stamp]


    def get_stamp_age(self, stamp):
        """
        Computes the age of a time stamp.
        """
        stamp_secs = stamp_tuple_to_secs(stamp)
        stamp_newest_secs = stamp_tuple_to_secs(self.stamp_newest)
        return stamp_newest_secs - stamp_secs


    def get_seq_age(self, seq):
        """
        Compute sequece age - handling 32bit uint rollover of seq counter.  Note, at 
        30Hz this should roll over for something like 4yrs, but you never know.
        """
        age0 = abs(self.seq_newest - seq)
        age1 = MAX_UINT32 - seq + self.seq_newest + 1
        return min([age0, age1])


    def publish_stitched_image(self):
        """
        Checks to see if the sequences to images buffer contains all required frames and if so 
        produces and publishes a stitched image.
        """
        # Check to see if we have all images for a given sequence number
        for seq, image_dict in sorted(self.seq_to_images.items()): 

            # If we have all images for the sequece - stitch into merged view
            if len(image_dict) == len(self.camera_list):

                t0 = rospy.Time.now() 

                for i, camera in enumerate(self.camera_list):

                    # Convert ros image to opencv image
                    ros_image = image_dict[camera] 
                    cv_image = self.bridge.imgmsg_to_cv(
                            ros_image,
                            desired_encoding="passthrough"
                            )
                    ipl_image = cv.GetImage(cv_image)

                    # Create stitced image if it doesn't exist yet
                    if self.stitched_image is None:
                        self.stitched_image = cv.CreateImage(
                                self.image_size,
                                ipl_image.depth,
                                ipl_image.channels
                                )
                    if i==0:
                        cv.Zero(self.stitched_image)

                    # Set image warping flags - if first fill rest of image with zeros
                    if self.is_first_write:
                        warp_flags = self.warp_flags | cv.CV_WARP_FILL_OUTLIERS
                        self.is_first_write = False
                    else:
                        warp_flags = self.warp_flags

                    # Warp into stitched image 
                    cv.SetImageROI(self.stitched_image, self.tf_data[camera]['roi'])

                    # Warp stitched image
                    cv.WarpPerspective(
                            ipl_image, 
                            self.stitched_image, 
                            self.tf_data[camera]['matrix'],
                            flags=warp_flags,
                            )
                    cv.ResetImageROI(self.stitched_image)

                    # Get max and min time stamps for stamp_info
                    stamp = ros_image.header.stamp
                    if i == 0:
                        stamp_max = stamp 
                        stamp_min = stamp
                    else:
                        stamp_max = stamp if stamp.to_sec() > stamp_max.to_sec() else stamp_max 
                        stamp_mim = stamp if stamp.to_sec() < stamp_min.to_sec() else stamp_min 

                # Convert stitched image to ros image
                stitched_ros_image = self.bridge.cv_to_imgmsg(
                        self.stitched_image,
                        encoding="passthrough"
                        )
                stitched_ros_image.header.seq = seq
                self.image_pub.publish(stitched_ros_image)
                self.image_and_seq_pub.publish(seq, stitched_ros_image)
                self.seq_pub.publish(seq)

                # Compute time stamp spread
                stamp_diff = stamp_max.to_sec() - stamp_min.to_sec()
                self.stamp_pub.publish(stamp_max, stamp_min, stamp_diff)
                
                t1 = rospy.Time.now()
                dt = t1.to_sec() - t0.to_sec()
                self.processing_dt_pub.publish(dt,1.0/dt)

                # Remove data from buffer
                del self.seq_to_images[seq]

            # Throw away any stale data in seq to images buffer
            seq_age = self.get_seq_age(seq)
            if seq_age > self.max_seq_age:
                try:
                    del self.seq_to_images[seq]
                except KeyError:
                    pass

    def run(self):

        while not rospy.is_shutdown(): 
            if self.seq_newest is None:
                continue
            if self.topic_type == 'sensor_msgs/Image':
                self.process_waiting_images() 
            self.publish_stitched_image()


# -----------------------------------------------------------------------------
def stamp_tuple_to_secs(stamp):
    """
    Converts a stamp tuple (secs,nsecs) to seconds.
    """
    return stamp[0] + stamp[1]*1.0e-9


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    if 0:
        # Old style - before frame drop correction
        topic_type = 'sensor_msgs/Image'
        topic_end = 'image_rect_skip'
    else:
        # New style - with frame drop correction
        topic_type = 'mct_msg_and_srv/SeqAndImage'
        topic_end = 'seq_and_image_corr_skip'

    region = sys.argv[1]
    node = ImageStitcher(region,topic_type=topic_type,topic_end=topic_end)
    node.run()
