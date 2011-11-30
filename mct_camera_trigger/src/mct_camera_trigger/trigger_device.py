import serial
import time

class CamTrigDev(serial.Serial):
    """
    Provides a simple serial interface to the JFRC multi-camera tracker
    hardware trigger device.
    """

    def __init__(self,*arg,**kwarg):
        super(CamTrigDev,self).__init__(*arg,**kwarg)
        time.sleep(1)

    def start(self,freq):
        """
        Start pulse output at the frequency given in (Hz)
        """
        t_sec = 1.0/float(freq)
        t_usx100 = t_sec*1.0e4
        self.write('start %d\n'%(t_usx100,))

    def stop(self):
        """
        Stop pulse output
        """
        self.write('stop\n')

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    """
    A simple test of the CamTrigDev
    """
    import sys

    port = '/dev/ttyUSB0'
    baudrate = 115200
    dev = CamTrigDev(port=port,baudrate=baudrate)
    if sys.argv[1] == 'start':
        freq = float(sys.argv[2])
        print 'start, freq = %f'%(freq,)
        dev.start(freq)
    else:
        print 'stop'
        dev.stop()

    dev.close()
