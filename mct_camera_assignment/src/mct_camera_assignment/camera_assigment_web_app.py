import roslib
roslib.load_manifest('mct_camera_assignment')
import rospy
import flask

from mct_camera_tools import mjpeg_servers

app = flask.Flask(__name__)

@app.route('/',methods=['GET','POST'])
def index():
    mjpeg_info_dict = mjpeg_servers.get_mjpeg_info_dict()

    if not hasattr(flask.g, 'camera_assignment'):
        flask.g.camera_assignment = {}
        for k in mjpeg_info_dict:
            flask.g.camera_assignment[k] = '--'

    if flask.request.method == 'POST':
        for k in mjpeg_info_dict:
            flask.g.camera_assignment[k] = flask.request.form[k]
    else:
        pass

    output = flask.render_template(
            'camera_view_table.html',
            mjpeg_info_dict=mjpeg_info_dict,
            camera_assignment=flask.g.camera_assignment,
            )
    return output

@app.route('/camerasbyguid')
def cameras_by_guid():
    mjpeg_info_dict = mjpeg_servers.get_mjpeg_info_dict()
    return flask.render_template('camera_list.html', mjpeg_info_dict=mjpeg_info_dict) 



# -----------------------------------------------------------------------------
if __name__ == '__main__':
    app.debug = True
    app.run()

