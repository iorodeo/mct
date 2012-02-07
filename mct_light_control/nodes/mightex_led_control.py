#!/usr/bin/env python
import roslib
roslib.load_manifest('mct_light_control')
import rospy
import threading
from mct_light_control import pymightled 

# Services
from mct_msg_and_srv.srv import LedEnable
from mct_msg_and_srv.srv import LedEnableResponse
from mct_msg_and_srv.srv import SetLedCurrent
from mct_msg_and_srv.srv import SetLedCurrentResponse
from mct_msg_and_srv.srv import GetLedSettings 
from mct_msg_and_srv.srv import GetLedSettingsResponse

class MightexLedControl(object):
    """
    Provides an interface to the Mightex Led current controllers.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.port = rospy.get_param('port', '/dev/ttyUSB1')
        self.default_imax = rospy.get_param('default_led_imax',1000)

        # Create device object and initialize
        self.dev = pymightled.LedController(self.port)
        self.initLedController()

        # Initialize node and setup services
        rospy.init_node('mightex_controller')

        rospy.Service(
                'led_enable',
                LedEnable,
                self.handle_led_enable,
                )

        rospy.Service(
                'set_led_current',
                SetLedCurrent,
                self.handle_set_led_current,
                )

        rospy.Service(
                'set_led_max_current',
                SetLedCurrent,
                self.handle_set_led_max_current,
                )

        rospy.Service(
                'get_led_settings',
                GetLedSettings,
                self.handle_get_led_settings,
                )

    def run(self):
        rospy.spin()

    # Service handlers
    # -------------------------------------------------------------------------
    def handle_led_enable(self,req):
        """
        Handles Led enable disable requests
        """
        flag = True
        chan = req.channel
        if not chan==0:
            if req.enable:
                self.enable(chan)
            else:
                self.disable(chan)
        else:
            if req.enable:
                self.enableAll()
            else:
                self.disableAll()
        return LedEnableResponse(True)

    def handle_set_led_current(self,req):
        """
        Handles request to set the led current for a given channel. Note,
        channel=0 is specail and means all channels.
        """
        chan = req.channel
        iset = req.current
        flag = True
        if not chan==0:
            # Turn on individual led channel
            try:
                self.setLedCurrent(chan,iset)
            except ValueError, e:
                flag = False
        else:
            # chan=0, turn pon all led channels
            try:
                self.setAllLedCurrent(iset)
            except ValueError, e:
                flag = False
        return SetLedCurrentResponse(flag)


    def handle_set_led_max_current(self,req):
        """
        Handles request to set the maximum allowed led current for a given
        channel. Note, channel=0 is special and means all channels.
        """
        chan = req.channel
        imax = req.current
        flag = True
        if not chan==0:
            try:
                self.setLedMaxCurrent(chan,imax)
            except ValueError, e:
                flag = False
        else:
            try:
                self.setAllLedMaxCurrent(imax)
            except ValueError, e:
                flag = False
        return SetLedCurrentResponse(flag)


    def handle_get_led_settings(self,req):
        """
        Handles requests for the led current for a given channel. Returns both
        the set point current iset and maximum current imax.
        """
        chan = req.channel
        mode = self.getLedMode(chan)
        if mode == 'disable':
            enable = False 
        else:
            enable = True
        imax, iset = self.getLedCurrent(chan)
        return GetLedSettingsResponse(enable,imax,iset)

    # Device access functions
    # --------------------------------------------------------------------------

    def setLedMaxCurrent(self,chan,imax):
        """
        Sets the maximum allowed current for the given channel
        """
        with self.lock:
            dummy, iset = self.dev.getNormalModeParams(chan)
            self.dev.setNormalModeParams(chan,imax,iset)

    def setAllLedMaxCurrent(self,imax):
        """
        Sets the maximum allowed current for all Led channels
        """
        for i in range(pymightled.NUM_CHANNELS):
            self.setLedMaxCurrent(i+1,imax)

    def setLedCurrent(self,chan,iset):
        """
        Sets the output set point current for the given channel
        """
        with self.lock:
            self.dev.setNormalModeCurrent(chan,iset)

    def setAllLedCurrent(self,iset):
        """
        Sets the ourput set point current for all channels
        """
        for i in range(pymightled.NUM_CHANNELS):
            self.setLedCurrent(i+1,iset)

    def enable(self,chan):
        """
        Enable current output on led given led channel
        """
        with self.lock:
            self.dev.setMode(chan,'normal')

    def enableAll(self):
        """
        Enable current output on all led channels
        """
        for i in range(pymightled.NUM_CHANNELS):
            self.enable(i+1)

    def disable(self,chan):
        """
        Disable current output on the given channel
        """
        with self.lock:
            self.dev.setMode(chan,'disable')

    def disableAll(self):
        """
        Disable current output on all channels
        """
        for i in range(pymightled.NUM_CHANNELS):        
            self.disable(i+1)

    def getLedCurrent(self,chan):
        """
        Gets the current set point and maximum current settings, iset and
        imax. 
        """
        with self.lock:
            imax, iset = self.dev.getNormalModeParams(chan)
        return imax, iset

    def getLedMode(self,chan):
        """
        Gets the operating mode of the specified led channel
        """
        with self.lock:
            mode = self.dev.getMode(chan)
        return mode

    def initLedController(self):
        """
        Initializes Led Device
        """
        self.disableAll()
        self.setAllLedCurrent(0)
        self.setAllLedMaxCurrent(self.default_imax)

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = MightexLedControl()
    node.run()
