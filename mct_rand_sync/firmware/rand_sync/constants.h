#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace constants 
{
    // Communications params
    enum {ringBufMaxSize=240};
    extern const long baudrate;
    extern const long timerPeriod;

    // IO Pins
    enum {numSyncSignal=3};
    enum {numGndPin=4};
    extern const int trigIntPin;
    extern const int trigIntNum;
    extern const int signalChangePin;
    extern const int syncSignalPin[numSyncSignal];
    extern const int gndPin[numGndPin];
    extern const int randomSeedAI;

    // Serial command and response ids
    enum 
    {
        cmdIdResetTrigCnt = 0,
        cmdIdGetSyncSignal,
        cmdIdGetTrigCnt,
    };
    enum
    {
        rspSuccess = 0,
        rspFail,
    };
    
}

#endif
