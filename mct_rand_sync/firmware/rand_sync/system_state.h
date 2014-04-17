#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "constants.h"
#include "ring_buffer.h"

class SyncData
{
    public:
        unsigned long trigCnt;
        uint8_t signal;
};

class SystemState
{
    public:
        SystemState();
        void initialize();
        void resetTrigCnt();
        void incrTrigCnt();
        unsigned long getTrigCnt();
        void resetTimerCnt();
        void incrTimerCnt();
        unsigned long getTimerCnt();
        void updateSyncSignal();
        void updateSyncDataBuf();
        bool getBufSyncSignal(unsigned long trigCnt, uint8_t &signal);
        unsigned int getCurSyncSignal();
        bool syncDataBufEmpty();
        bool syncDataBufFull();


    private:
        unsigned long trigCnt_; 
        unsigned long timerCnt_;
        uint8_t syncSignal_;

        uint8_t signalOutPort_;
        uint8_t changePinMask_;
        uint8_t syncPinMask_[constants::numSyncSignal];

        RingBuffer<SyncData,constants::ringBufMaxSize> syncDataBuf_;
};



#endif
