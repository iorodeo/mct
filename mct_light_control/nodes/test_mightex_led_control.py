import roslib
roslib.load_manifest('mct_light_control')
import rospy

from mct_light_control import led_control

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import sys

    # Get command argument
    name = sys.argv[1]
    cmd = sys.argv[2]
    cmd = cmd.lower()

    if cmd == 'set-current':
        # Set current commands
        chan = int(sys.argv[3])
        value = int(sys.argv[4])
        led_control.set_led_current(name,chan,value)
    elif cmd == 'set-max-current':
        # Set max current commands
        chan = int(sys.argv[3])
        value = int(sys.argv[4])
        led_control.set_led_max_current(name,chan,value)
    elif cmd == 'enable':
        # Enable/disable commands
        chan = int(sys.argv[3])
        value = sys.argv[4]
        value = value.lower()
        if value == 'true':
            led_control.led_enable(name,chan,True)
        elif value == 'false':
            led_control.led_enable(name,chan,False)
        else:
            print('\nError: unknown enable command: {0}\n'.format(value))
    elif cmd == 'settings':
        # Get current command
        chan = int(sys.argv[3])
        enable, imax, iset = led_control.get_led_settings(name,chan)
        print('enable: {0}, imax: {1}, iset: {2}'.format(enable,imax,iset))
    else:
        print('\nError: unrecognised command: {0}\n'.format(cmd))


