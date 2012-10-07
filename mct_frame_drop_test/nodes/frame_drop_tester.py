#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_frame_drop_test')
import rospy

import os
import os.path
import tempfile
import subprocess
import functools
import threading
import mct_introspection

from mct_xml_tools import launch
from mct_msg_and_srv.msg import BlobData

class Frame_Drop_Tester(object):

    def __init__(self, camera_number_0, camera_number_1, output_file):
        self.ready = False
        self.tester_popen = None
        self.lock =  threading.Lock()
        self.camera_0 = 'camera_{0}'.format(camera_number_0)
        self.camera_1 = 'camera_{0}'.format(camera_number_1)
        self.output_file = output_file
        self.fid = open(self.output_file,'w')
        self.camera_list = [self.camera_0, self.camera_1]
        self.tmp_dir = tempfile.gettempdir()
        self.launch_file = os.path.join(self.tmp_dir,'frame_drop_test.launch')
        rospy.on_shutdown(self.clean_up)
        rospy.init_node('frame_drop_tester')
        self.launch_tester()
        self.seq_to_blob_data = {}
        blob_data_topics = self.get_blob_data_topics()

        blob_data_sub = {}
        for topic in blob_data_topics:
            camera = topic.split('/')[2]
            handler = functools.partial(self.blob_data_handler,camera)
            blob_data_sub[camera] = rospy.Subscriber(topic,BlobData,handler)

        self.ready = True

    def blob_data_handler(self,camera,data):
        if not self.ready:
            return
        with self.lock:
            try:
                self.seq_to_blob_data[data.image_seq][camera] = data
            except KeyError:
                self.seq_to_blob_data[data.image_seq] = {camera:data}

    def get_blob_data_topics(self):
        blob_data_topics = []
        while len(blob_data_topics) < 2:
            blob_data_topics = mct_introspection.find_topics_w_ending('blob_data')
            blob_data_topics = [t for t in blob_data_topics if 'frame_drop_test' in t]
        return blob_data_topics

    def launch_tester(self):
        launch.create_frame_drop_test_launch(self.launch_file,self.camera_0,self.camera_1)
        self.tester_popen = subprocess.Popen(['roslaunch',self.launch_file])

    def kill_tester(self):
        if self.tester_popen is not None:
            self.tester_popen.send_signal(subprocess.signal.SIGINT)
            self.tester_popen = None
            try:
                os.remove(self.launch_file)
            except OSError, e:
                rospy.logwarn('Error removing frame skipper launch file: {0}'.format(str(e)))

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                for seq, blob_data in sorted(self.seq_to_blob_data.items()):
                    if len(blob_data) == 2:
                        num0 = blob_data[self.camera_0].number_of_blobs
                        num1 = blob_data[self.camera_1].number_of_blobs
                        print('seq:', seq)
                        print('ok: ', num0==num1)
                        print('number_of_blobs:')
                        print('  {0} {1}'.format(self.camera_0,num0))
                        print('  {0} {1}'.format(self.camera_1,num1))
                        print('---- ')
                        print()
                        self.fid.write('{0} {1} {2}\n'.format(seq,num0,num1))
                        del self.seq_to_blob_data[seq]

    def clean_up(self):
        self.kill_tester()
        self.fid.close()

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import sys
    camera_number_0 = int(sys.argv[1])
    camera_number_1 = int(sys.argv[2])
    try:
        output_file = sys.argv[3]
    except IndexError:
        output_file = 'frame_drop_data.txt'

    node = Frame_Drop_Tester(camera_number_0, camera_number_1,output_file)
    node.run()
