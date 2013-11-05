#include "constants.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

namespace constants
{
    // Communications params
    const long baudrate = 115200;
    const long timerPeriod = 1000000;

    // IO Pins
    const int trigIntPin = 2;
    const int trigIntNum = 0;
    const int signalChangePin = 5;
    const int syncSignalPin[numSyncSignal] = {7,9,11};
    const int gndPin[numGndPin] = {6,8,10,12};
    const int randomSeedAI = A7;

}
