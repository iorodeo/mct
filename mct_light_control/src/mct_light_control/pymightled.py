"""
Copyright 2010  IO Rodeo Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
import serial
import time

MODE_STR2INT = {
        'disable' : 0,
        'normal'  : 1,
        'strobe'  : 2,
        'trigger' : 3,
        }

POLARITY_STR2INT = {
        'rising'  : 0,
        'falling' : 1,
        }

RESET_SLEEP_DT = 4.0
STORE_TIMEOUT = 2.0
MAX_REPEAT = 99999999
MAX_CURRENT = 1000
NUM_PROFILE_STEPS = 128
NUM_CHANNELS = 4
DEBUG = False 

class LedController(serial.Serial):
    """
    Provides a serial interface to the Mightex Sirius SLC-XXXX-S/U multi-channel
    LED Controllers.
    """

    def __init__(self,port, timeout=0.1):
        super(LedController,self).__init__(
                port=port,
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                timeout=timeout,
                )
        self.num_channels = NUM_CHANNELS
        # Put device in known echo state
        self._echoOff(checkResponse=False)

    def getMode(self,chan):
        """
        Returns the current working mode for the given channel
        """
        chan = self._checkChan(chan)
        resp = self._writeCmd('?MODE {0}'.format(chan))
        modeInt = int(resp[0][1])
        modeStr = findKey(MODE_STR2INT,modeInt)
        return modeStr

    def setMode(self,chan,mode):
        """
        Sets the current working mode for the given channel. Allowed modes
        are 'disable', 'normal', 'strobe' and 'trigger'.
        """
        chan = self._checkChan(chan)
        modeNum = MODE_STR2INT[mode]
        resp = self._writeCmd('MODE {0} {1}'.format(chan,modeNum))

    # Normal mode methods
    # -------------------------------------------------------------------------

    def setNormalModeParams(self,chan,imax,iset):
        """
        Sets the maximum allowed current, when in normal mode, for the given
        channel.

        chan = channel number 1,2,3 or 4
        imax = maximum allowed current.
        iset = set point current

        Note, requires a call to setMode(chan,'normal') to take effect even if
        the device is already in normal mode.
        """
        chan = self._checkChan(chan)
        imax = self._checkCurrent(imax)
        iset = self._checkCurrent(iset)
        if iset > imax:
            raise ValueError, 'iset must be <= imax'
        resp = self._writeCmd('NORMAL {0} {1} {2}'.format(chan,imax,iset))

    def setNormalModeCurrent(self,chan,iset):
        """
        Sets the working current, when in normal mode, for the given channel.
        Takes effect immediately if in normal mode.

        chan = channel number 1,2,3 or 4
        iset = set point current
        """
        chan = self._checkChan(chan)
        iset = self._checkCurrent(iset)
        self._writeCmd('CURRENT {0} {1}'.format(chan,iset))

    def getNormalModeParams(self,chan):
        """
        Returns the working and maximum current setting for the current channel
        when in normal mode.
        """
        chan = self._checkChan(chan)
        resp = self._writeCmd('?CURRENT {0}'.format(chan))
        valueList = resp[0].split()
        imax = int(valueList[-2])
        iset = int(valueList[-1])
        return imax, iset

    # Strobe mode methods
    # ------------------------------------------------------------------------- 

    def setStrobeModeParams(self,chan,imax,repeat):
        """
        Sets the strobe mode parameters. 

        chan = channel number 1,2,3 or 4
        imax = maximum current for strobe mode
        repeat = repeat count for running the profile. Either an integer in the
        range [1,99999999] or the string 'forever'. Note, the value 9999 is
        special and means forever
        """
        chan = self._checkChan(chan)
        imax = self._checkCurrent(imax)
        repeat = self._checkRepeat(repeat)
        self._writeCmd('STROBE {0} {1} {2}'.format(chan,imax,repeat))

    def setStrobeModeProfile(self,chan,step,iset,tset):
        """
        Sets the profile for strobe mode.
        
        chan = channel number 1,2,3 or 4
        step = step number of profile, integer in ragne [0,127]
        iset = set point current for this step
        tset = time for this step in us

        Note, while there can be as many as 128 steps I don't think all device
        support this many.  I could only set 2 step values (0 and 1) for the
        SLC-SA04-U/S device that I used for development.
        """
        chan = self._checkChan(chan)
        step = self._checkStep(step)
        iset = self._checkCurrent(iset)
        tset = self._checkTime(tset)
        self._writeCmd('STRP {0} {1} {2} {3}'.format(chan,step,iset,tset))

    def getStrobeModeParams(self,chan):
        """
        Gets the strobe mode parameters for the given channel. Returns 
        imax = maximum current for strobe mode
        repeat = repeat count for the running profile
        """
        chan = self._checkChan(chan) 
        resp = self._writeCmd('?STROBE {0}'.format(chan))
        valueStr = resp[0].split()
        imax = int(valueStr[0][1:])
        repeat = int(valueStr[1])
        return imax, repeat

    def getStrobeModeProfile(self,chan):
        """
        Gets the strobe mode profile parameters for the given channel.

        returns a list of (iset,tset) pairs.
        """
        chan = self._checkChan(chan)
        resp = self._writeCmd('?STRP {0}'.format(chan))
        profileValues = self._getProfileValues(resp)
        return profileValues

    # Trigger mode methods
    # -------------------------------------------------------------------------

    def setTriggerModeParams(self,chan,imax,polarity):
        """
        Sets the trigger mode parameters for the given channel.

        chan = channel number 1,2,3 or 4
        imax = maximum allowed current for trigger mode.
        polarity = polarity of trigger 'rising', 'falling'
        """
        chan = self._checkChan(chan)
        imax = self._checkCurrent(imax)
        polarity = self._checkPolarity(polarity)
        polarityInt = POLARITY_STR2INT[polarity]
        self._writeCmd('TRIGGER {0} {1} {2}'.format(chan,imax,polarityInt))

    def setTriggerModeProfile(self,chan,step,iset,tset):
        """
        Sets the trigger mode profile parameters for the given channel.

        chan = channel number 1,2,3 or 4
        step = profile step number range [0,127]
        iset = set point current for this step
        tset = the time for this step in us

        Note, for some devices there may not be the full 128 step values for
        the trigger profile. In particular for the SLC-SA04-U/S device that I
        used for development only 2 step value could be set. I'm not sure the
        2nd step value does anything either, but I didn't have enough time 
        to play around with it.
        """
        chan = self._checkChan(chan)
        step = self._checkStep(step)
        iset = self._checkCurrent(iset)
        tset = self._checkTime(tset)
        resp = self._writeCmd('TRIGP {0} {1} {2} {3}'.format(chan,step,iset,tset))

    def getTriggerModeParams(self,chan):
        """
        Gets the trigger mode parameters for the given channel.
        """
        chan = self._checkChan(chan)
        resp = self._writeCmd('?TRIGGER {0}'.format(chan))
        valueList = resp[0].split()
        imax = int(valueList[0][1:])
        polarityInt = int(valueList[1])
        polarity = findKey(POLARITY_STR2INT,polarityInt)
        return imax, polarity

    def getTriggerModeProfile(self,chan):
        """
        Gets the trigger mode profile values for the given channel.

        returns a list of (iset,tset) pairs.
        """
        chan = self._checkChan(chan)
        resp = self._writeCmd('?TRIGP {0}'.format(chan))
        profileValues = self._getProfileValues(resp)
        return profileValues


    # Methods for other commands 
    # -------------------------------------------------------------------------

    def reset(self,sleep=False):
        """
        Perform a soft reset of the device. Note, it seems to take about 4sec
        for the device to reset.  You will need to close the serial connection
        and reopen it again. 
        """
        self._writeCmd('Reset')
        if sleep:
            time.sleep(RESET_SLEEP_DT)

    def restoreDefaults(self,store=False):
        """
        Restore the device's mode and all related parameters to its factory default.
        Values are not written to non-volatile memory . If this is desired the store
        the optional keyword argument store can be set to true. 
        """
        self._writeCmd('RESTOREDEF')
        if store:
            self.store()

        # Set all channels to the mode given in the defaults seems to be required 
        # in order of the changes to take effect.
        for i in range(1,self.num_channels+1):
            mode = self.getMode(i)
            self.setMode(i,mode)

    def store(self):
        """
        Store the current settings in non-volatile memory.
        """
        # Use a longer timeout for the store command as it takes a while to 
        # Write to non-volatile memory. Save the origianl value. 
        original_timeout = self.timeout
        self.timeout = STORE_TIMEOUT
        self._writeCmd('STORE')
        self.timeout = original_timeout

    def printSettings(self):
        """
        Prints current parameters 
        """
        print
        print(self.getDeviceInfo())
        print
        for i in range(1,self.num_channels+1):

            mode = self.getMode(i)

            imax, iset = self.getNormalModeParams(i)
            print('chan: {0}'.format(i))
            print('  mode: {0}'.format(mode))
            print('  normal mode parameters')
            print('    imax: {0}'.format(imax))
            print('    iset: {0}'.format(iset))

            imax, repeat = self.getStrobeModeParams(i)
            print('  strobe mode parameters')
            print('    imax: {0}'.format(imax))
            print('    repeat: {0}'.format(repeat))
            profileValues = self.getStrobeModeProfile(i)
            print('  strobe mode profile')
            for j,values in enumerate(profileValues):
                iset,tset = values
                print('    step {0}'.format(j))
                print('      iset: {0}'.format(iset)) 
                print('      tset: {0}'.format(tset))

            imax, polarity =  self.getTriggerModeParams(i)
            print('  trigger mode parameters')
            print('    imax: {0}'.format(imax))
            print('    polarity: {0}'.format(polarity))
            profileValues = self.getTriggerModeProfile(i)
            print('  trigger mode profile')
            for j,values in enumerate(profileValues):
                iset,tset = values
                print('    step {0}'.format(j))
                print('      iset: {0}'.format(iset)) 
                print('      tset: {0}'.format(tset))

            print('')
    
    def getDeviceInfo(self):
        """
        Queries the device for information ..  device type, firmware version,
        serial number, etc. Returns as a string.
        """
        resp = self._writeCmd('DEVICEINFO')
        infoStr = resp[0].strip()
        return infoStr

    def _echoOff(self,checkResponse=True):
        """
        Turns off echo mode
        """
        self.echo = False 
        self._writeCmd('ECHOOFF',checkResponse=checkResponse)

    def _echoOn(self):
        """
        Turns on echo mode - useful for debugging
        """
        self.echo = True 
        self._writeCmd('ECHOON')

    def _getProfileValues(self,resp):
        """
        Extract the list of profile values from the device's response.
        """
        profileValues = []
        for i,valueStr in enumerate(resp):
            valueList =  valueStr.split()
            iset = valueList[0]
            if iset[0] == '#':
                iset = iset[1:]
            iset = int(iset)
            tset = int(valueList[1])
            profileValues.append((iset,tset))
        profileValues.pop()
        return profileValues

    def _checkCurrent(self,value):
        """
        Checks the given current value. Converts to an integer and 
        verifies that it is between 0 and MAX_CURRENT.
        """
        value = int(value)
        if value < 0 or value > MAX_CURRENT:
            raise ValueError, 'current must be >= 0 or < 1000'
        return value

    def _checkChan(self,chan):
        """
        Checks the given channel values. Converts to an integer and
        verifies that it is in [1,2,3,4]
        """
        chan = int(chan)
        if chan < 1 or chan > self.num_channels:
            raise ValueError, 'chan must be between 1 and 4'
        return chan

    def _checkRepeat(self,repeat):
        """
        Checks the repeat value. Converts to an integer and verifies that is in
        the allowed range. In addition checks for the string 'forever' and
        replaces is with the special integer value 9999 which means forever.
        """
        try:
            repeat = int(repeat)
        except ValueError,e:
            if repeat.lower() == 'forever':
                repeat = 9999
            else:
                raise ValueError, str(e)
        if repeat < 1 or repeat > MAX_REPEAT:
            raise ValueError, 'repeat must be > 1 and <= {0}'.format(MAX_REPEAT)
        return repeat

    def _checkStep(self,step):
        """
        Check the step value. Converts to an integer and verifies that is within 
        the allowed range.
        """
        step = int(step)
        if step < 0 or step >= NUM_PROFILE_STEPS:
            raise ValueError, 'step must be in range [0,{0}'.format(NUM_PROFILE_STEPS-1)
        return step

    def _checkTime(self,t):
        """
        Checks the time value. Converts to an integer and verifies that is is
        greater than 0
        """
        t = int(t)
        if t < 0:
            raise ValueError, 'time must be > 0'
        return t

    def _checkPolarity(self,polarity):
        """
        Checks the polarity value. Converts to lower case and makes sure it is one 
        of the possible allowed values.
        """
        polarity = polarity.lower()
        if not polarity in POLARITY_STR2INT:
            raise ValueError, "polarity must be either 'rising' or  'falling'"
        return polarity

    def _writeCmd(self,cmd,checkResponse=True):
        """
        Writes a command to the LED controller and receives a response.
        """
        if DEBUG:
            print('cmd: {0}'.format(cmd)) 

        self.write('{0}\r\n'.format(cmd))
        resp = self.readlines()

        if DEBUG:
            print('rsp: {0}'.format(resp))
            
        if self.echo:
            # If we are in echo mode separate out the echo from the 
            # response
            echo = resp[0]
            resp = resp[1:]

        if checkResponse:
            # Check the response for errors - not done
            pass
        return resp


def findKey(d,val):
    """
    Find a dictionary key for the given value
    """
    return [k for k,v in d.iteritems() if v == val][0]


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    port = '/dev/ttyUSB0'
    chan = 1
    dev = LedController(port)

    if 0:
        dev.printSettings()
        dev.close()

    if 1:
        for i in range(1,5):
            dev.setMode(i,'disable')

    if 0:
        dev.setNormalModeParams(1,1000,0)
        dev.setNormalModeParams(3,1000,0)
        dev.setNormalModeCurrent(1,300)
        dev.setNormalModeCurrent(3,50)
        dev.setMode(1,'normal')
        dev.setMode(3,'normal')

    if 0:
        dev.setTriggerModeParams(chan,800,'rising')
        dev.setTriggerModeProfile(chan,0,400,int(1e5))
        dev.setTriggerModeProfile(chan,1,400,int(1e5))
        dev.setMode(chan,'trigger')
        dev.printSettings()
        dev.store()


    if 0:
        # Pulse mode test
        dev.setStrobeModeParams(chan,500,'forever')
        dev.setStrobeModeProfile(chan,0,500,int(1e5))
        dev.setStrobeModeProfile(chan,1,0,int(1e5))
        dev.setMode(chan,'strobe')

    if 0:
        print('Restoring device to factory defaults')
        dev.restoreDefaults(store=True)
        

    dev.close()
    del dev

