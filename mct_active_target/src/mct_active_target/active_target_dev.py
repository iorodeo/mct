from __future__ import print_function
import serial
import time

RESET_SLEEP_DT = 2

class ActiveTargetDev(serial.Serial):
    """
    Provides a simple serial interface to the arduino controlled active calibration 
    target.
    """

    def __init__(self,target_info):
        port, baudrate, square, ledArraySize, maxPowerInt,  = self.extract_info(target_info)
        super(ActiveTargetDev,self).__init__(port=port, baudrate=baudrate)
        self.ledArraySize = ledArraySize
        self.maxPowerInt = maxPowerInt
        self.square = square
        time.sleep(RESET_SLEEP_DT)

    def sendCmd(self,cmd):
        self.write('{0}\n'.format(cmd))

    def off(self):
        """
        Turn all led's off.
        """
        cmd = '[0]'
        self.sendCmd(cmd)

    def led(self,i,j,power):
        """
        Light up led i,j with the given power level.
        """
        powerInt = int(power)
        if i < 0 or i >= self.ledArraySize[0]:
            msg = 'led index must be between 0 and {0}'.format(self.ledArraySize[0])
            raise ValueError, msg
        if j < 0 or j >= self.ledArraySize[1]:
            msg = 'led index must be between 0 and {0}'.format(self.ledArraySize[1])
            raise ValueError, msg
        if powerInt<0 or powerInt>=self.maxPowerInt:
            msg = 'power must be between 0 and {0}'.format(self.maxPowerInt)
            raise ValueError, msg
        cmd = '[1,{0},{1},{2}]'.format(i,j,powerInt)
        self.sendCmd(cmd)

    def pattern(self):
        cmd = '[2]'
        self.sendCmd(cmd)

    def all(self):
        cmd = '[3]'
        self.sendCmd(cmd)

    def extract_info(self,target_info):
        port = target_info['port']
        baudrate = target_info['baudrate']
        square = target_info['square']
        ledArraySize = tuple([int(x) for x in target_info['size'].split('x')])
        maxPowerInt = target_info['max_power']
        return port, baudrate, square, ledArraySize, maxPowerInt

        

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import roslib
    roslib.load_manifest('mct_active_target')
    import rospy
    from mct_utilities import file_tools

    target_info = file_tools.read_target_info('active')
    dev = ActiveTargetDev(target_info)

    if 0:
        dev.led(5,3,10)
    if 1:
        dev.pattern()

    raw_input('to quit press enter')

    dev.off()
    dev.close()

