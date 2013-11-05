#include <Streaming.h>
#include <TimerOne.h>
#include <SerialReceiver.h>
#include <util/atomic.h>
#include "constants.h"
#include "ring_buffer.h"
#include "system_state.h"
#include "serial_handler.h"

SystemState systemState;
SerialHandler comm;

void onExternalTrigger();
void onTimerOverflow();

void setup()
{
    Serial.begin(constants::baudrate);
    systemState.initialize();
    Timer1.initialize();
    Timer1.attachInterrupt(onTimerOverflow, constants::timerPeriod);
    attachInterrupt(constants::trigIntNum, onExternalTrigger, RISING);
}


void loop()
{
    comm.processInput();
}

void onExternalTrigger()
{
    systemState.updateSyncDataBuf();
    systemState.incrTrigCnt();
}

void onTimerOverflow()
{
    systemState.updateSyncSignal();
    systemState.incrTimerCnt();
}

