import serial
import time

MAX_POWER_INT = 50 
LED_ARRAY_SIZE = (7,7)
RESET_SLEEP_DT = 2

class ActiveTarget(serial.Serial):
    """
    Provides a simple serial interface to the arduino controlled active calibration 
    target.
    """

    def __init__(self,*arg,**kwarg):
        super(ActiveTarget,self).__init__(*arg,**kwarg)
        self.ledArraySize = LED_ARRAY_SIZE
        self.maxPowerInt = MAX_POWER_INT
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

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    port = '/dev/ttyUSB2'
    baudrate = 9600
    dev = ActiveTarget(port=port,baudrate=baudrate)

    if 1:
        dev.led(5,3,10)
    else: 
        dev.pattern()

    raw_input('to quit press enter')

    dev.off()
    dev.close()

