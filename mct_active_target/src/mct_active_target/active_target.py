from __future__ import print_function
import roslib
roslib.load_manifest('mct_active_target')
import rospy
import time
from mct_msg_and_srv.srv import ActiveTargetCmd
from mct_msg_and_srv.srv import ActiveTargetInfo

def active_target_cmd(command,ind0=0,ind1=0,power=5):
    rospy.wait_for_service('active_target_cmd')
    proxy = rospy.ServiceProxy('active_target_cmd',ActiveTargetCmd)
    resp = proxy(command,ind0,ind1,power)
    status, message = resp.status, resp.message
    return status, message

def active_target_info():
    rospy.wait_for_service('active_target_info')
    proxy = rospy.ServiceProxy('active_target_info',ActiveTargetInfo)
    resp = proxy()
    return resp.array_size_n, resp.array_size_m, resp.max_power

def set_pattern():
    active_target_cmd('pattern')

def set_led(i,j,power):
    active_target_cmd('led',i,j,power)

def off():
    active_target_cmd('off')


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import time

    print('active_target_info')
    led_n, led_m, max_power = active_target_info()
    print('led_n:     {0}'.format(led_n))
    print('led_m:     {0}'.format(led_m))
    print('max_power: {0}'.format(max_power))
    print()
    time.sleep(1.0)

    print('set_led')
    for i in range(0,led_n):
        for j in range(0,led_m):
            print(' (i,j) = ({0},{1})'.format(i,j))
            set_led(i,j,20)
            time.sleep(0.25)
    print()

    print('set_pattern')
    set_pattern()
    print()

    time.sleep(5)

    print('off')
    off()
    print()




