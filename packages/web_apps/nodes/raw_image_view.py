#!/usr/bin/env python
import roslib
roslib.load_manifest('web_apps')
import rospy
import os
import flask
import redis
import atexit
import tracker_camera_tools

use_streamer = False 
app = flask.Flask(__name__)
db = redis.Redis('localhost',db=0)

# Start the throttling nodes and the mjpeg streamer
if use_streamer:
    streamer = tracker_camera_tools.MJPEG_Streamer()
    streamer.start()

@app.route("/")
def index():
    hostname = os.uname()[1]
    cameras= tracker_camera_tools.find_cameras()
    ports = tracker_camera_tools.get_camera_ports(cameras)
    cameras_and_ports = zip(cameras,ports)
    return flask.render_template("streams.html",hostname=hostname,cameras_and_ports=cameras_and_ports)

@app.route("/<camera>")
def camera(camera):
    hostname = os.uname()[1]
    port = tracker_camera_tools.get_camera_ports([camera])[0]
    return flask.render_template("camera.html",hostname=hostname,camera=camera,port=port)

def exit_func():
    if use_streamer:
        streamer.stop()
    for k in db.keys('*'):
        db.delete(k)

atexit.register(exit_func)

# ---------------------------------------------------------------------------------------
if __name__ == '__main__':
    if 1:
        app.run(host='0.0.0.0')
    else:
        app.debug = True
        app.run()



