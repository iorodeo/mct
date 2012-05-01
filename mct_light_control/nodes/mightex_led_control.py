#!/usr/bin/env python
import roslib
roslib.load_manifest('mct_light_control')
import rospy
import threading
from mct_light_control import pymightled 
from mct_utilities import file_tools



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
        self.params = file_tools.read_mightex_params()
        
        #self.port = '/dev/mightex-serial' 
        self.default_max_current = 1000 

        # Create device object and initialize

        #self.dev = pymightled.LedController(self.port)
        self.dev = {} 
        self.initLedControllers()

        # Initialize node and setup services
        rospy.init_node('mightex_controller')
        rospy.on_shutdown(self.onShutdown)

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
        name = req.name
        chan = req.channel
        if not chan==0:
            if req.enable:
                self.enable(name,chan)
            else:
                self.disable(name,chan)
        else:
            if req.enable:
                self.enableAll(name)
            else:
                self.disableAll(name)
        return LedEnableResponse(True)

    def handle_set_led_current(self,req):
        """
        Handles request to set the led current for a given channel. Note,
        channel=0 is specail and means all channels.
        """
        name = req.name
        chan = req.channel
        iset = req.current
        flag = True
        if not chan==0:
            # Turn on individual led channel
            try:
                self.setLedCurrent(name,chan,iset)
            except ValueError, e:
                flag = False
        else:
            # chan=0, turn pon all led channels
            try:
                self.setAllLedCurrent(name,iset)
            except ValueError, e:
                flag = False
        return SetLedCurrentResponse(flag)


    def handle_set_led_max_current(self,req):
        """
        Handles request to set the maximum allowed led current for a given
        channel. Note, channel=0 is special and means all channels.
        """
        name = req.name
        chan = req.channel
        imax = req.current
        flag = True
        if not chan==0:
            try:
                self.setLedMaxCurrent(name,chan,imax)
            except ValueError, e:
                flag = False
        else:
            try:
                self.setAllLedMaxCurrent(name,imax)
            except ValueError, e:
                flag = False
        return SetLedCurrentResponse(flag)


    def handle_get_led_settings(self,req):
        """
        Handles requests for the led current for a given channel. Returns both
        the set point current iset and maximum current imax.
        """
        name = req.name
        chan = req.channel
        mode = self.getLedMode(name,chan)
        if mode == 'disable':
            enable = False 
        else:
            enable = True
        imax, iset = self.getLedCurrent(name,chan)
        return GetLedSettingsResponse(enable,imax,iset)

    # Device access functions
    # --------------------------------------------------------------------------

    def setLedMaxCurrent(self,name,chan,imax):
        """
        Sets the maximum allowed current for the given channel
        """
        with self.lock:
            dummy, iset = self.dev[name].getNormalModeParams(chan)
            self.dev[name].setNormalModeParams(chan,imax,iset)

    def setAllLedMaxCurrent(self,name,imax):
        """
        Sets the maximum allowed current for all Led channels
        """
        for i in range(pymightled.NUM_CHANNELS):
            self.setLedMaxCurrent(name,i+1,imax)

    def setLedCurrent(self,name,chan,iset):
        """
        Sets the output set point current for the given channel
        """
        with self.lock:
            self.dev[name].setNormalModeCurrent(chan,iset)

    def setAllLedCurrent(self,name,iset):
        """
        Sets the ourput set point current for all channels
        """
        for i in range(pymightled.NUM_CHANNELS):
            self.setLedCurrent(name,i+1,iset)

    def enable(self,name,chan):
        """
        Enable current output on given device name and led channel
        """
        with self.lock:
            self.dev[name].setMode(chan,'normal')

    def enableAll(self,name):
        """
        Enable current output on all led channels for the given device name
        """
        for i in range(pymightled.NUM_CHANNELS):
            self.enable(name,i+1)

    def disable(self,name,chan):
        """
        Disable current output on the given channel and device name
        """
        with self.lock:
            self.dev[name].setMode(chan,'disable')

    def disableAll(self,name):
        """
        Disable current output on all channels on the given device name
        """
        for i in range(pymightled.NUM_CHANNELS):        
            self.disable(name,i+1)

    def getLedCurrent(self,name, chan):
        """
        Gets the current set point and maximum current settings, iset and
        imax. 
        """
        with self.lock:
            imax, iset = self.dev[name].getNormalModeParams(chan)
        return imax, iset

    def getLedMode(self,name,chan):
        """
        Gets the operating mode of the specified led channel
        """
        with self.lock:
            mode = self.dev[name].getMode(chan)
        return mode

    def initLedControllers(self):
        """
        Initializes Led Device
        """
        for name, dev_params in self.params.iteritems():
            self.dev[name] = pymightled.LedController(dev_params['port'])
            for i in range(pymightled.NUM_CHANNELS):
                chan_name = 'channel_{0}'.format(i+1)
                try:
                    chan_params = dev_params[chan_name]
                    current = chan_params['current']
                    enabled = chan_params['enabled']
                except KeyError:
                    current = 0
                    enabled = False

                self.disable(name,i+1)
                self.setLedMaxCurrent(name,i+1,self.default_max_current)
                self.setLedCurrent(name,i+1,current)
                if enabled:
                    self.enable(name,i+1)
                else:
                    self.disable(name,i+1)

    def onShutdown(self):
        for name in self.params:
            self.setAllLedCurrent(name,0)
            self.disableAll(name)


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    node = MightexLedControl()
    node.run()
