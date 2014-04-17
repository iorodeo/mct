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

    // Get singal output port and pin masks for change and sync signals
    // Note, change and sync signals are assumed to be on the same port. 
    signalOutPort_ = digitalPinToPort(constants::signalChangePin);
    changePinMask_ = digitalPinToBitMask(constants::signalChangePin);
    for (int i=0; i<constants::numSyncSignal; i++)
    {
        syncPinMask_[i] = digitalPinToBitMask(constants::syncSignalPin[i]);
    }
}

void SystemState::resetTrigCnt()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        trigCnt_ = 0;
        syncDataBuf_.empty();
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
    volatile uint8_t *outReg = portOutputRegister(signalOutPort_);
    uint8_t outValue = 0x0;

    // Change pin mask based on current value
    if (*outReg & changePinMask_)
    {
        outValue &= ~ changePinMask_;
    }
    else
    {
        outValue |= changePinMask_;
    }

    // Get random synchronization signals 
    syncSignal_ = 0xff & uint8_t(random());
    for (uint8_t i=0; i<constants::numSyncSignal; i++)
    {
        uint8_t syncBit = 0x1 & (syncSignal_ >> i);
        if (syncBit)
        {
            outValue |= syncPinMask_[i];
        }
        else
        {
            outValue &= ~syncPinMask_[i];
        }
    }

    // Actually set output values (simultaneous)
    *outReg = outValue;
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



