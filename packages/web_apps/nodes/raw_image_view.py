#!/usr/bin/env python
import roslib
roslib.load_manifest('web_apps')
import rospy
import os
import sys
import flask
import redis
import atexit
import tracker_camera_tools
import netifaces

use_streamer = False 
external_iface = 'eth0'
internal_iface = 'eth1'

app = flask.Flask(__name__)
db = redis.Redis('localhost',db=0)

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

def get_ip_addr(iface):
    """
    Returns the IP address for the given interface.
    """
    ifaddresses = netifaces.ifaddresses(iface)
    ip = ifaddresses[2][0]['addr']
    return ip

def exit_func():
    if use_streamer:
        streamer.stop()
    for k in db.keys('*'):
        db.delete(k)

atexit.register(exit_func)


# ---------------------------------------------------------------------------------------
if __name__ == '__main__':

    try:
        arg = sys.argv[1].lower()
        if arg == 'true':
            debug_flag = True
        else:
            debug_flag = False
    except IndexError:
        debug_flag = True

    if debug_flag:
        print 'starting in debug mode'
        app.debug = True
        app.run()
    else:
        print 'starting in normal mode'
        app.run(host='0.0.0.0')




