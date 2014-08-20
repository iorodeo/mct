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
    const int gndPin[numGndPin] = {5,6,7,10,12};
    const int randomSeedAI = A7;

    // Change and sync signal pins - must all be on the same physical port
    const int signalChangePin = 8;
    const int syncSignalPin[numSyncSignal] = {9,11,13}; 

}
