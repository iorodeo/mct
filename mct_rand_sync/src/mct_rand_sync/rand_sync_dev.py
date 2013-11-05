from __future__ import print_function
import serial
import time


class RandSyncDev(serial.Serial):

    CMD_RESET_TRIG_CNT = 0
    CMD_GET_SYNC_SIGNAL = 1
    CMD_GET_TRIG_CNT = 2

    RSP_SUCCESS = 0
    RSP_FAIL = 1

    RESET_SLEEP_DT = 2.0
    NUM_SYNC_SIGNAL = 3

    def __init__(self,port,baudrate):
        super(RandSyncDev,self).__init__(port=port, baudrate=baudrate)
        time.sleep(self.RESET_SLEEP_DT)

    def sendCmd(self,cmd):
        self.write('{0}\n'.format(cmd))
        time.sleep(0.001)
        rsp = self.readline()
        rsp = rsp.strip()
        if rsp[0] == '[' and rsp[-1] == ']':
            rsp = rsp[1:-1]
        else:
            raise IOError, 'invalid response from device'
        rsp = rsp.split(',')
        status = int(rsp[0])
        if status == self.RSP_FAIL:
            try:
                errMsg = rsp[1]
            except IndexError:
                errMsg = 'error message missing'
            raise IOError, errMsg 
        return rsp

    def resetTrigCnt(self):
        cmd = '[{0}]'.format(self.CMD_RESET_TRIG_CNT)
        self.sendCmd(cmd)

    def getSyncSignal(self,trigCnt):
        if trigCnt < 0:
            raise ValueError, 'trigCnt must be >= 0'
        cmd = '[{0},{1}]'.format(self.CMD_GET_SYNC_SIGNAL,trigCnt)
        rsp = self.sendCmd(cmd)
        syncSignal = []
        for i in range(self.NUM_SYNC_SIGNAL):
            try:
                sig = int(rsp[i+1])
            except IndexError:
                raise IOError, 'missing sync signal'
            except TypeError:
                raise IOError, 'unable to convert sync signal to int'
            syncSignal.append(sig)
        return tuple(syncSignal)


    def getTrigCnt(self):
        cmd = '[{0}]'.format(self.CMD_GET_TRIG_CNT)
        rsp = self.sendCmd(cmd)
        try:
            trigCnt = int(rsp[1])
        except IndexError:
            raise IOError, 'trigger count missing'
        except TypeError:
            raise IOError, 'unable to conver trigger count to int'
        return trigCnt


# -----------------------------------------------------------------------------

if __name__ == '__main__':

    port = '/dev/ttyUSB2'
    baudrate = 115200

    dev = RandSyncDev(port,baudrate)
    dev.resetTrigCnt()
    
    #while 1:
    for i in range(2000);
        trigCnt = dev.getTrigCnt()
        reqTrigCnt = trigCnt - 10
        print('trigCnt: {0}'.format(trigCnt))
        print('reqTrigCnt: {0}'.format(reqTrigCnt))
        try:
            syncSignal = dev.getSyncSignal(reqTrigCnt)
            print('syncSignal: {0}'.format(syncSignal))
        except IOError,e:
            print('IOError: {0}'.format(str(e)))
        except ValueError, e:
            print('ValueError: {0}'.format(str(e)))
        print()
        time.sleep(0.02)



