import roslib
roslib.load_manifest('mct_redis')
import redis
import time
import cPickle as pickle
from sensor_msgs.msg import CameraInfo

db = redis.Redis('localhost',db=2)

while 1:
    data_str = db.get('camera2_info')
    try:
        data = pickle.loads(data_str)
    except TypeError:
        data = None
    print data
    print
    time.sleep(1.0/30.0)


