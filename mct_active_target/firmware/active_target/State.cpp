#include <util/atomic.h>
#include "State.h"
#include "Streaming.h"

State::State(uint8_t _mode, uint8_t _ledPin, uint8_t _ledPower) {
    mode = _mode;
    ledPin = _ledPin;
    ledPower = _ledPower;
    timerCount = 0;
}

void State::initialize() {
    for (int i=0; i<ledArrayN; i++) {
        for (int j=0; j<ledArrayM; j++) {
            pinMode(ledArray[i][j], OUTPUT);
        }
    }
}

void State::setModeOff() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        mode = modeOff;
    }
    for (int i=0; i<ledArrayN; i++) {
        for (int j=0; j<ledArrayM; j++) {
            digitalWrite(ledArray[i][j],LOW);
        }
    }
}

void State::setModeSingleLed(uint8_t i, uint8_t j, uint8_t newLedPower) {
    uint8_t newLedPin;
    if ((i < ledArrayN) && (j < ledArrayM) && (newLedPower < timerCountMax)) {
        newLedPin = ledArray[i][j];
        setModeSingleLed(newLedPin,newLedPower);
    }
}

void State::setModeSingleLed(uint8_t newLedPin, uint8_t newLedPower) {
    setModeOff();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        digitalWrite(ledPin,LOW);
        mode = modeSingleLed;
        ledPin = newLedPin;
        ledPower = newLedPower;
    }
}

void State::setModePattern() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        mode = modePattern;
    }
    for (int i=0; i<ledArrayN; i++) {
        for (int j=0; j<ledArrayM; j++) {
            if ((i==j) || (j==0) || (i==ledArrayN-1) || (j==ledArrayM-1)) {
                digitalWrite(ledArray[i][j],HIGH);
            }
            else {
                digitalWrite(ledArray[i][j],LOW);
            }
        }
    }
}

void State::updateTimerCount() {
    timerCount++;
    if (timerCount >= timerCountMax) {
        timerCount = 0;
    }
}

void State::updateSingleLedOutput() {
    if (mode == modeSingleLed) {
        if (timerCount < ledPower) {
            digitalWrite(ledPin,HIGH); 
        }
        else {
            digitalWrite(ledPin,LOW);
        }
    }
}
