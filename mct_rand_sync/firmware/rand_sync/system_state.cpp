#include "system_state.h"
#include <Streaming.h>
#include <util/atomic.h>

SystemState::SystemState()
{
    resetTrigCnt();
    resetTimerCnt();
}

void SystemState::initialize()
{
    // Initialize random number generator w/ 'random' seed
    randomSeed(analogRead(constants::randomSeedAI));

    // Setup IO Pins
    pinMode(constants::trigIntPin,INPUT);
    pinMode(constants::signalChangePin, OUTPUT);
    digitalWrite(constants::signalChangePin,LOW);
    for (int i=0; i<constants::numSyncSignal; i++)
    {
        pinMode(constants::syncSignalPin[i],OUTPUT);
    }
    for (int i=0; i<constants::numGndPin; i++)
    {
        pinMode(constants::gndPin[i],OUTPUT);
        pinMode(constants::gndPin[i],LOW);
    }
    updateSyncSignal();
}

void SystemState::resetTrigCnt()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        trigCnt_ = 0;
    }
}

void SystemState::incrTrigCnt()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    { 
        trigCnt_++;
    }
}

unsigned long SystemState::getTrigCnt()
{
    unsigned long trigCntTemp;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        trigCntTemp = trigCnt_;
    }
    return trigCntTemp;
}

void SystemState::resetTimerCnt()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        timerCnt_ = 0;
    }
}

void SystemState::incrTimerCnt()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        timerCnt_++;
    }
}

unsigned long SystemState::getTimerCnt()
{
    unsigned long timerCntTemp;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        timerCntTemp = timerCnt_;
    }
    return timerCntTemp;
}

void SystemState::updateSyncSignal()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        uint8_t changeVal = digitalRead(constants::signalChangePin);
        digitalWrite(constants::signalChangePin, !changeVal);
        syncSignal_ = 0xff & uint8_t(random());
        for (int i=0; i<constants::numSyncSignal; i++)
        {
            uint8_t syncBit = 0x1 & (syncSignal_ >> i);
            digitalWrite(constants::syncSignalPin[i], syncBit);
        }
    }
}


void SystemState::updateSyncDataBuf()
{
    SyncData data;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        data.trigCnt = trigCnt_;
        data.signal = syncSignal_;
        syncDataBuf_.add(data);
    }
}


bool SystemState::getBufSyncSignal(unsigned long trigCnt, uint8_t &signal)
{
    bool status = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (!syncDataBuf_.isEmpty())
        {
            SyncData dataNewest = syncDataBuf_.getItemNewest();
            SyncData dataOldest = syncDataBuf_.getItemOldest();
            if ((trigCnt >= dataOldest.trigCnt) && (trigCnt <= dataNewest.trigCnt))
            {
               unsigned int age = (unsigned int)(dataNewest.trigCnt - trigCnt);
               SyncData data;
               status = syncDataBuf_.getItemByAge(age,data);
               signal = data.signal;
            }
        }
    }
    return status;
}

unsigned int SystemState::getCurSyncSignal()
{
    return syncSignal_;
}


bool SystemState::syncDataBufEmpty()
{
    return syncDataBuf_.isEmpty();
}

bool SystemState::syncDataBufFull()
{
    return syncDataBuf_.isFull();
}



