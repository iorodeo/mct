#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_tracking_2d')
import rospy
import sys
import functools
import threading
import math
import cv
import numpy
import Image as PILImage
import ImageDraw as PILImageDraw
import ImageFont as PILImageFont
import mct_introspection
import time

from cv_bridge.cv_bridge import CvBridge 
from mct_utilities import file_tools
from mct_transform_2d import transform_2d

from sensor_msgs.msg import Image
from mct_msg_and_srv.msg import Point2d
from mct_msg_and_srv.msg import ThreePointTracker
from mct_msg_and_srv.msg import ThreePointTrackerRaw
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

class ThreePointTracker_Synchronizer:
    """
    Synchronization node for all three point tracker in a given tracking region.
    """

    def __init__(self,region,max_seq_age=200):
        self.lock = threading.Lock()

        self.region = region
        regions_dict = file_tools.read_tracking_2d_regions()
        self.camera_list = regions_dict[region]
        self.camera_list.sort()
        self.create_camera_to_tracking_dict()
        self.latest_seq = None
        self.max_seq_age = max_seq_age
        self.tracking_pts_pool = {}

        # Color and font for tracking points image
        self.magenta = (255,255,0)
        self.cv_text_font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX, 0.6, 0.6, thickness=0)

        # Font for PIL tracking info image
        self.info_image_size = (180,100)
        self.font = PILImageFont.truetype("/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-B.ttf", 16)

        # Get transforms from cameras to tracking and stitched image planes
        self.tf2d = transform_2d.Transform2d()

        self.bridge = CvBridge()
        self.ready = False
        rospy.init_node('three_point_tracker_synchronizer', log_level=rospy.DEBUG)

        # Subscribe to raw tracking pts topics
        self.tracking_pts_sub = {}
        
        for camera, topic in self.camera_to_tracking.iteritems():
            handler = functools.partial(self.tracking_pts_handler, camera)
            self.tracking_pts_sub[camera] = rospy.Subscriber(
                    topic,
                    ThreePointTrackerRaw,
                    handler
                    )

        # Create publishers
        self.tracking_pts_pub = rospy.Publisher('tracking_pts', ThreePointTracker)
        self.image_tracking_pts = None
        self.image_tracking_pts_pub = rospy.Publisher('image_tracking_pts', Image)
        self.image_tracking_info_pub = rospy.Publisher('image_tracking_info', Image)

        # Setup reset service - needs to be called anytime the camera trigger is  
        # stopped - before it is restarted. Empties buffers of images and sequences. 
        self.reset_srv = rospy.Service('reset_tracker_synchronizer', Empty, self.reset_handler)
        self.ready = True
        
    def create_camera_to_tracking_dict(self):
        """
        Creates a dictionary relating the cameras in the tracking region to their
        corresponding three point tracker nodes.
        """
        self.camera_to_tracking = {}
        for camera in self.camera_list:
            camera_fullpath_topic = mct_introspection.get_camera_fullpath_topic(camera)
            tracking_pts_topic = '{0}/tracking_pts'.format(camera_fullpath_topic)
            self.camera_to_tracking[camera] = tracking_pts_topic

    def reset_handler(self,req):
        """
        Reset service handler. Empties the tracking_pts_pool.
        """
        with self.lock:
            self.tracking_pts_pool = {}
        return EmptyResponse()

    def tracking_pts_handler(self,camera,msg):
        """
        Handler for messages from the individual tracker nodes. Sticks the tracking
        point data into a dictionary by sequence number and camera.
        """
        if not self.ready:
            return 
        with self.lock:
            self.latest_seq = msg.data.seq
            try:
                self.tracking_pts_pool[msg.data.seq][camera] = msg
            except KeyError:
                self.tracking_pts_pool[msg.data.seq] = {camera: msg}

    def process_tracking_pts(self,tracking_pts_dict):
        """
        Determines whether the object has been found in any of the three point
        trackers for the region. If is has been found than best tracking point
        data is selected in a winner takes all fashion. The best tracking point
        data is that which is nearest to the center of the image on camera upon
        which is was captured. 
        """

        # Get time stamp (secs, nsecs) - always from the same camera to avoid jumps due to 
        # possible system clock differences.
        time_camera = self.camera_list[0]
        time_camera_secs = tracking_pts_dict[time_camera].data.secs
        time_camera_nsecs = tracking_pts_dict[time_camera].data.nsecs

        # Get list of messages in which the object was found
        found_list = [msg for msg in tracking_pts_dict.values() if msg.data.found]
        tracking_pts_msg = ThreePointTracker()

        if found_list:

            # Object found - select object with largest ROI or if the ROIs are of equal size 
            # select the object whose distance to center of the # image is the smallest 
            found_list.sort(cmp=tracking_pts_sort_cmp)
            best = found_list[0]
            camera = best.data.camera

            # Get coordintates of points in tracking and stitching planes
            best_points_array = numpy.array([(p.x,p.y) for p in best.data.points])
            pts_anchor_plane = self.tf2d.camera_pts_to_anchor_plane(camera, best_points_array)
            pts_stitching_plane = self.tf2d.camera_pts_to_stitching_plane(camera, best_points_array) 
            pts_anchor_plane = [Point2d(p[0],p[1]) for p in list(pts_anchor_plane)]
            pts_stitching_plane = [Point2d(p[0],p[1]) for p in list(pts_stitching_plane)]

            # Get orientation angle and mid point of object in anchor and stitching planes 
            angle = get_angle(pts_anchor_plane)
            midpt_anchor_plane = get_midpoint(pts_anchor_plane)
            midpt_stitching_plane = get_midpoint(pts_stitching_plane)
            #self.camera_fail = max([(err,cam) for cam, err in self.max_error_by_camera.items()])[1]

            # Get size of tracking points image in the anchor (tracking) plane 
            roi = best.data.roi
            x0, x1 = roi[0], roi[0] + roi[2]
            y0, y1 = roi[1], roi[1] + roi[3]
            bndry_camera = [(x0,y0), (x1,y0), (x1,y1), (x0,y1)]
            bndry_camera_array = numpy.array(bndry_camera)
            bndry_anchor = self.tf2d.camera_pts_to_anchor_plane(camera,bndry_camera_array)
            bndry_stitching = self.tf2d.camera_pts_to_stitching_plane(camera,bndry_camera_array)
            bndry_anchor = [tuple(x) for x in list(bndry_anchor)]
            bndry_stitching = [tuple(x) for x in list(bndry_stitching)]
            dx1 = abs(bndry_anchor[1][0] - bndry_anchor[0][0])
            dx2 = abs(bndry_anchor[3][0] - bndry_anchor[2][0])
            dy1 = abs(bndry_anchor[2][1] - bndry_anchor[1][1])
            dy2 = abs(bndry_anchor[3][1] - bndry_anchor[0][1])
            dx_max = max([dx1, dx2])
            dy_max = max([dy1, dy2])
            dim_max = max([dx_max, dy_max])

            # Convert tracking points image to opencv image.
            image_tracking_pts = self.bridge.imgmsg_to_cv(best.image,desired_encoding="passthrough")
            image_tracking_pts = cv.GetImage(image_tracking_pts)
            image_size = cv.GetSize(image_tracking_pts)
            image_dim_max = max(image_size)

            # Get matrix for homography from camera to  anchor plane
            tf_matrix = self.tf2d.get_camera_to_anchor_plane_tf(camera)

            # Shift for local ROI
            tf_shift = numpy.array([ 
                [1.0, 0.0, roi[0]],
                [0.0, 1.0, roi[1]],
                [0.0, 0.0,    1.0],
                ])
            tf_matrix = numpy.dot(tf_matrix, tf_shift)

            # Get scaling factor
            shift_x = -min([x for x,y in bndry_anchor])
            shift_y = -min([y for x,y in bndry_anchor])
            scale_factor = float(image_dim_max)/dim_max

            # Scale and shift transform so that homography maps the tracking points
            # sub-image into a image_size image starting at coord. 0,0
            tf_shift_and_scale = numpy.array([
                [scale_factor,           0.0,   scale_factor*shift_x],
                [         0.0,  scale_factor,   scale_factor*shift_y], 
                [         0.0,           0.0,                    1.0],
                ])
            tf_matrix = numpy.dot(tf_shift_and_scale, tf_matrix)

            # Warp image using homography
            image_tracking_pts_mod = cv.CreateImage(
                    image_size, 
                    image_tracking_pts.depth, 
                    image_tracking_pts.channels
                    )
            cv.WarpPerspective(
                    image_tracking_pts, 
                    image_tracking_pts_mod,
                    cv.fromarray(tf_matrix),
                    cv.CV_INTER_LINEAR | cv.CV_WARP_FILL_OUTLIERS,
                    )

            # Add sequence number to image
            message = '{0}'.format(best.data.seq)
            cv.PutText(image_tracking_pts_mod, message, (2,15), self.cv_text_font, self.magenta)
            self.image_tracking_pts = self.bridge.cv_to_imgmsg(image_tracking_pts_mod,encoding="passthrough")

            # Create tracking points message
            tracking_pts_msg.seq = best.data.seq
            tracking_pts_msg.secs = time_camera_secs
            tracking_pts_msg.nsecs = time_camera_nsecs
            tracking_pts_msg.camera = camera
            tracking_pts_msg.found = True
            tracking_pts_msg.angle = angle
            tracking_pts_msg.angle_deg = (180.0*angle)/math.pi
            tracking_pts_msg.midpt_anchor_plane = midpt_anchor_plane
            tracking_pts_msg.midpt_stitching_plane = midpt_stitching_plane
            tracking_pts_msg.pts_anchor_plane = pts_anchor_plane
            tracking_pts_msg.pts_stitching_plane = pts_stitching_plane
            tracking_pts_msg.bndry_anchor_plane = [Point2d(x,y) for x,y in bndry_anchor]
            tracking_pts_msg.bndry_stitching_plane =[Point2d(x,y) for x,y in bndry_stitching]
        else:
            sample = tracking_pts_dict.values()[0]
            tracking_pts_msg.seq = sample.data.seq
            tracking_pts_msg.secs = time_camera_secs
            tracking_pts_msg.nsecs = time_camera_nsecs
            tracking_pts_msg.camera = '' 
            tracking_pts_msg.found = False

        # Create tracking info image
        pil_info_image = PILImage.new('RGB', self.info_image_size,(255,255,255))
        draw = PILImageDraw.Draw(pil_info_image)
        text_list = []

        label = 'Found:'
        value = '{0}'.format(tracking_pts_msg.found)
        unit = ''
        text_list.append((label,value,unit))

        label = 'Angle:'
        value = '{0:+3.1f}'.format(180.0*tracking_pts_msg.angle/math.pi)
        unit = 'deg'
        text_list.append((label,value,unit))

        label = 'Pos X:'
        value = '{0:+1.3f}'.format(tracking_pts_msg.midpt_anchor_plane.x)
        unit =  'm'
        text_list.append((label,value,unit))
            #self.camera_fail = max([(err,cam) for cam, err in self.max_error_by_camera.items()])[1]

        label = 'Pos Y:'
        value = '{0:+1.3f}'.format(tracking_pts_msg.midpt_anchor_plane.y)
        unit = 'm'
        text_list.append((label,value,unit))
        
        for i, text in enumerate(text_list):
            label, value, unit = text
            draw.text((10,10+20*i),label,font=self.font,fill=(0,0,0))
            draw.text((70,10+20*i),value,font=self.font,fill=(0,0,0))
            draw.text((125,10+20*i),unit,font=self.font,fill=(0,0,0))
        cv_info_image = cv.CreateImageHeader(pil_info_image.size, cv.IPL_DEPTH_8U, 3)
        cv.SetData(cv_info_image, pil_info_image.tostring())

        # Convert to a rosimage and publish
        info_rosimage = self.bridge.cv_to_imgmsg(cv_info_image,'rgb8')
        self.image_tracking_info_pub.publish(info_rosimage)

        # Publish messages
        self.tracking_pts_pub.publish(tracking_pts_msg)
        if self.image_tracking_pts is not None:
            self.image_tracking_pts_pub.publish(self.image_tracking_pts)

    def run(self):
        """
        Node main loop - consolidates the tracking points messages by
        acquisition sequence number and proccess them by passing them to the
        process_tracking_pts function.
        """
        while not rospy.is_shutdown():

            with self.lock:

                #t0 = time.time()
                #print('len pool:', len(self.tracking_pts_pool))
                for seq, tracking_pts_dict in sorted(self.tracking_pts_pool.items()):
                    #print(' ', seq)

                    # Check if we have all the tracking data for the given sequence number
                    if len(tracking_pts_dict) == len(self.camera_list):
                        self.process_tracking_pts(tracking_pts_dict)
                        del self.tracking_pts_pool[seq]

                    # Throw away tracking data greater than the maximum allowed age  
                    if self.latest_seq - seq > self.max_seq_age:
                        #print('Thowing away: ', seq)
                        del self.tracking_pts_pool[seq]
                #t1 = time.time()
                #print('loop time:', t1-t0)


# Utility functions
# -----------------------------------------------------------------------------
def tracking_pts_sort_cmp(msg_x, msg_y):
    """
    Comparison function for sorting the the tracking pts messages. Used for sorting
    the based on ROI arean and distance. 
    """
    roi_x = msg_x.data.roi_src
    roi_y = msg_y.data.roi_src
    area_roi_x = roi_x[2]*roi_x[3]
    area_roi_y = roi_y[2]*roi_y[3]
    if area_roi_x < area_roi_y:
        return 1
    elif area_roi_x > area_roi_y:
        return -1
    else:
        dist_x = msg_x.data.distance
        dist_y = msg_y.data.distance
        if dist_x > dist_y:
            return 1
        elif dist_y > dist_x:
            return -1
        else:
            return 0

def get_midpoint(point_list):
    """
    Retuns the mid-point of the sorted 2d points of the three-point tracking object.
    """
    x_mid = 0.5*(point_list[0].x + point_list[-1].x)
    y_mid = 0.5*(point_list[0].y + point_list[-1].y)
    return Point2d(x_mid, y_mid)

def get_angle(point_list):
    """
    Returns the orientation angle given the sort 2d point of the three-point tracking
    object.
    """
    p = point_list[0]
    q = point_list[-1]
    dy = q.y - p.y
    dx = q.x - p.x
    angle = math.atan2(dy, dx)
    return angle

# -----------------------------------------------------------------------------
if __name__  == '__main__':
    region = sys.argv[1]
    node = ThreePointTracker_Synchronizer(region)
    node.run()



