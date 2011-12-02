import roslib
roslib.load_manifest('mct_light_control')
import rospy

# Services
from mct_msg_and_srv.srv import LedEnable
from mct_msg_and_srv.srv import SetLedCurrent
from mct_msg_and_srv.srv import GetLedSettings

def ledEnable(chan,enable):
    rospy.wait_for_service('led_enable')
    try:
        proxy = rospy.ServiceProxy('led_enable',LedEnable)
        proxy(chan,enable)
    except rospy.ServiceException, e:
        print('service call failed: {0}'.format(str(e)))


def setLedCurrent(chan,iset):
    rospy.wait_for_service('set_led_current')
    try:
        proxy = rospy.ServiceProxy('set_led_current',SetLedCurrent)
        proxy(chan,iset)
    except rospy.ServiceException, e:
        print('service call failed: {0}'.format(str(e)))

def setLedMaxCurrent(chan,imax):
    rospy.wait_for_service('set_led_max_current')
    try:
        proxy = rospy.ServiceProxy('set_led_max_current',SetLedCurrent)
        proxy(chan,imax)
    except rospy.ServiceException, e:
        print('service call failed: {0}'.format(str(e)))

def getLedSettings(chan):
    rospy.wait_for_service('get_led_settings')
    try:
        proxy = rospy.ServiceProxy('get_led_settings',GetLedSettings)
        resp = proxy(chan)
        enable = resp.enable
        imax = resp.max_current
        iset = resp.current 
    except rospy.ServiceException, e:
        print('service call failed: {0}'.format(str(e)))
        imax, iset = None, None
    return enable, imax, iset


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import sys

    # Get command argument
    cmd = sys.argv[1]
    cmd = cmd.lower()

    if cmd == 'seti':
        # Set current commands
        chan = int(sys.argv[2])
        value = int(sys.argv[3])
        setLedCurrent(chan,value)
    elif cmd == 'setimax':
        # Set max current commands
        chan = int(sys.argv[2])
        value = int(sys.argv[3])
        setLedMaxCurrent(chan,value)
    elif cmd == 'enable':
        # Enable/disable commands
        chan = int(sys.argv[2])
        value = sys.argv[3]
        value = value.lower()
        if value == 'true':
            ledEnable(chan,True)
        elif value == 'false':
            ledEnable(chan,False)
        else:
            print('\nError: unknown enable command: {0}\n'.format(value))
    elif cmd == 'settings':
        # Get current command
        chan = int(sys.argv[2])
        enable,imax, iset = getLedSettings(chan)
        print('enable: {0}, imax: {1}, iset: {2}'.format(enable,imax,iset))
    else:
        print('\nError: unrecognised command: {0}\n'.format(cmd))


