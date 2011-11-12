#!/usr/bin/env python

# Tornado web server imports
from tornado.wsgi import WSGIContainer
from tornado.httpserver import HTTPServer
from tornado.ioloop import IOLoop

# Flask web framework imports
import flask
import flaskext.sijax

#  ROS imports
import roslib
roslib.load_manifest('web_apps')
import rospy

# Other imports
import os
import sys
import redis
import atexit
import tracker_camera_tools
import netifaces
import time
import random
from sensor_msgs.msg import CameraInfo
import cPickle as pickle

use_streamer = False 
external_iface = 'eth0'
internal_iface = 'eth1'

# Create the application and initialize sijax
app = flask.Flask(__name__)

# The path where you want the extension to create the needed javascript files
# DON'T put any of your files in this directory, because they'll be deleted!
app.config["SIJAX_STATIC_PATH"] = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')

# You need to point Sijax to the json2.js library if you want to support
# browsers that don't support JSON natively (like IE <= 7)
app.config["SIJAX_JSON_URI"] = '/home/albert/pyenv/test/lib/python2.7/site-packages/sijax/js/json2.js'

flaskext.sijax.init_sijax(app)

# Set up redis key,value stores
topic_db = redis.Redis('localhost',db=2)
state_db = redis.Redis('localhost',db=0)

# Start the throttling nodes and the mjpeg streamer
if use_streamer:
    streamer = tracker_camera_tools.MJPEG_Streamer()
    streamer.start()

@app.route("/")
def index():
    """
    Index page - currently just a bounce point for development.
    """
    external_ip = get_ip_addr(external_iface)
    internal_ip = get_ip_addr(internal_iface)
    uname = os.uname()
    hostname = uname[1]
    system = uname[0] + uname[2]
    camera_list = tracker_camera_tools.find_cameras()
    system_info = {
            'System:': system,
            'Hostname:': hostname,
            'External IP:': external_ip,
            'Internal IP:': internal_ip,
            'External iface:': external_iface,
            'Internal iface:':  internal_iface,
            'Server:' : state_db.get('server'),
            }

    kwargs = {
            'system_info': system_info,
            'camera_list': camera_list,
            }
    return flask.render_template('index.html',**kwargs)


@app.route("/allcameras")
def allcameras():
    """
    Shows live streams for all currently defined cameras.
    """
    ip = get_ip_addr(external_iface)
    cameras= tracker_camera_tools.find_cameras()
    ports = tracker_camera_tools.get_camera_ports(cameras)
    cameras_and_ports = zip(cameras,ports)
    kwargs = {
            'host': ip,
            'cameras_and_ports': cameras_and_ports,
            }
    return flask.render_template("allcameras.html",**kwargs)

@app.route("/<camera>")
def camera(camera):
    """
    Shows a live stream for the given camera
    """
    ip = get_ip_addr(external_iface)
    port = tracker_camera_tools.get_camera_ports([camera])[0]
    camera_list = tracker_camera_tools.find_cameras()
    kwargs = {
            'host': ip,
            'camera': camera,
            'port': port,
            'camera_list': camera_list,
            }
    return flask.render_template("camera.html",**kwargs)

@app.route("/topics")
def topics():
    """
    Returns a list of all currently published ROS topics
    """
    topic_list = rospy.get_published_topics('/')
    topic_list = [topic[0] for topic in topic_list]
    topic_list.sort()
    kwargs = {'topic_list': topic_list}
    return flask.render_template('topics.html',**kwargs)

@app.route("/topics/bycamera", methods=['GET', 'POST']) 
def camera_topics():
    """
    Allows user to view topics by camera
    """
    camera_list = tracker_camera_tools.find_cameras()

    # Get selected camera
    if flask.request.method == 'POST':
        selected_camera = flask.request.form['camera']
    else:
        selected_camera = camera_list[0]

    # Display ROS topic information
    topic_list = rospy.get_published_topics('/')
    topic_list = [topic[0] for topic in topic_list]
    camera_topic_list = []
    for topic in topic_list:
        top_name = topic.split('/')[1]
        if top_name == selected_camera:
            camera_topic_list.append(topic)
       
    kwargs = {
            'camera_list': camera_list,
            'selected_camera': selected_camera,
            'camera_topic_list': camera_topic_list,
            'form':  str(flask.request.form),
            }
    return flask.render_template('camera_topics.html',**kwargs)

# sijax tests
# --------------------------------------------------------------------------------

def sijax_test_handler(obj_response, sleep_time):
    while 1:
        data_str = topic_db.get('camera1_info')
        data = pickle.loads(data_str)
        seq = 'seq: %s'%(data.header.seq,)
        obj_response.html('#seq', seq)
        yield obj_response
        time.sleep(sleep_time)

@flaskext.sijax.route(app, '/sijax_test')
def sijax_test():
    if flask.g.sijax.is_sijax_request:
        flask.g.sijax.register_comet_callback('do_work', sijax_test_handler)
        return flask.g.sijax.process_request()
    return flask.render_template('sijax_test.html')

def comet_do_work_handler(obj_response, sleep_time):
    for i in range(6):
        width = '%spx' % (i * 80)
        obj_response.css('#progress', 'width', width)
        obj_response.html('#progress', width)
        yield obj_response
        if i != 5:
            # The sleep here kills tornado and I think makes the example only work 
            # in a single instance of the builtin sever.  
            time.sleep(sleep_time)

@flaskext.sijax.route(app, "/sijax_comet")
def sijax_comet():
    if flask.g.sijax.is_sijax_request:
        flask.g.sijax.register_comet_callback('do_work', comet_do_work_handler)
        return flask.g.sijax.process_request()
    return flask.render_template('comet.html')

# -----------------------------------------------------------------------------
def get_ip_addr(iface):
    """
    Returns the IP address for the given interface.
    """
    ifaddresses = netifaces.ifaddresses(iface)
    ip = ifaddresses[2][0]['addr']
    return ip

def exit_func():
    """
    Cleanup on shutdown
    """
    if use_streamer:
        streamer.stop()
    for k in state_db.keys('*'):
        state_db.delete(k)

atexit.register(exit_func)


# ---------------------------------------------------------------------------------------
if __name__ == '__main__':

    try:
        server = sys.argv[1].lower()
    except IndexError:
        server = 'debug' 

    if server=='debug':
        state_db.set('server','builtin -- debug mode')
        print 'starting builtin server -- debug mode'
        app.debug = True
        app.run()
    elif server=='standard':
        state_db.set('server', 'builtin - standard mode')
        print 'starting builtin server -- standard mode'
        app.run(host='0.0.0.0')
    elif server == 'tornado':
        print 'starting tornado server'
        state_db.set('server', 'tornado')
        http_server = HTTPServer(WSGIContainer(app))
        http_server.listen(5000)
        IOLoop.instance().start()
    else:
        print 'unknow server option -- goodbye'



