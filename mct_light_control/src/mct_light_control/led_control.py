import roslib
roslib.load_manifest('mct_light_control')
import rospy

# Services
from mct_msg_and_srv.srv import LedEnable
from mct_msg_and_srv.srv import SetLedCurrent
from mct_msg_and_srv.srv import GetLedSettings

def led_enable(name,chan,enable):
    rospy.wait_for_service('led_enable')
    try:
        proxy = rospy.ServiceProxy('led_enable',LedEnable)
        proxy(name,chan,enable)
    except rospy.ServiceException, e:
        print('service call failed: {0}'.format(str(e)))


def set_led_current(name,chan,iset):
    rospy.wait_for_service('set_led_current')
    try:
        proxy = rospy.ServiceProxy('set_led_current',SetLedCurrent)
        proxy(name,chan,iset)
    except rospy.ServiceException, e:
        print('service call failed: {0}'.format(str(e)))

def set_led_max_current(name,chan,imax):
    rospy.wait_for_service('set_led_max_current')
    try:
        proxy = rospy.ServiceProxy('set_led_max_current',SetLedCurrent)
        proxy(name,chan,imax)
    except rospy.ServiceException, e:
        print('service call failed: {0}'.format(str(e)))

def get_led_settings(name,chan):
    rospy.wait_for_service('get_led_settings')
    try:
        proxy = rospy.ServiceProxy('get_led_settings',GetLedSettings)
        resp = proxy(name,chan)
        enable = resp.enable
        imax = resp.max_current
        iset = resp.current 
    except rospy.ServiceException, e:
        print('service call failed: {0}'.format(str(e)))
        imax, iset = None, None
    return enable, imax, iset
