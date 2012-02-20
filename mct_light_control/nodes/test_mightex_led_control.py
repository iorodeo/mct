import roslib
roslib.load_manifest('mct_light_control')
import rospy

from mct_light_control import led_control

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
        led_control.set_led_current(chan,value)
    elif cmd == 'setimax':
        # Set max current commands
        chan = int(sys.argv[2])
        value = int(sys.argv[3])
        led_control.set_led_max_current(chan,value)
    elif cmd == 'enable':
        # Enable/disable commands
        chan = int(sys.argv[2])
        value = sys.argv[3]
        value = value.lower()
        if value == 'true':
            led_control.led_enable(chan,True)
        elif value == 'false':
            led_control.led_enable(chan,False)
        else:
            print('\nError: unknown enable command: {0}\n'.format(value))
    elif cmd == 'settings':
        # Get current command
        chan = int(sys.argv[2])
        enable, imax, iset = led_control.get_led_settings(chan)
        print('enable: {0}, imax: {1}, iset: {2}'.format(enable,imax,iset))
    else:
        print('\nError: unrecognised command: {0}\n'.format(cmd))


