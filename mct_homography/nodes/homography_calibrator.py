import roslib
roslib.load_manifest('mct_homography')
import roslib
import sys

# Messages
from sensor_msgs.msg import Image

class HomographyCalibrator(object):

    def __init__(self,topic):
        self.topic = topic

    def run(self):
        rospy.spin()


# -----------------------------------------------------------------------------
if __name___ == '__main__':
    topic = sys.argv[1]
    node = HomographyCalibrator(topic)
    node.run()
