#ifndef _STATE_H_
#define _STATE_H_
#include "constants.h"

class State {
    public:
        State(uint8_t _mode=modeOff, uint8_t _ledPin=ledArray[0][0], uint8_t _ledPower=1);
        uint8_t mode;
        uint8_t ledPin;
        uint8_t ledPower;
        uint8_t timerCount;
        void initialize();
        void setModeOff();
        void setModeSingleLed(uint8_t newLedPin, uint8_t newLedPower);
        void setModeSingleLed(uint8_t i, uint8_t j, uint8_t newLedPower);
        void setModePattern();
        void setModeAll();
        void updateTimerCount();
        void updateSingleLedOutput();
};

#endif
