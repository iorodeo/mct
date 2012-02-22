#include <util/atomic.h>
#include <Streaming.h>
#include <TimerOne.h>
#include "State.h"
#include "SerialReceiver.h"
#include "constants.h"
#include "SerialHandler.h"


SerialHandler handler;
State sysState;

void setup() {
    Serial.begin(baudrate);
    sysState.initialize();
    Timer1.initialize(timerPeriod);
    Timer1.attachInterrupt(timerInterrupt);
}

void loop() {
    handler.processMsg();
}

void timerInterrupt() {
    sysState.updateSingleLedOutput();
    sysState.updateTimerCount();
}
