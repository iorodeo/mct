#include "serial_handler.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Streaming.h>
#include "constants.h"

SerialHandler::SerialHandler() {}

void SerialHandler::processInput()
{
    while (Serial.available() > 0) {
        process(Serial.read());
        if (messageReady()) {
            switchYard();
            reset();
        }
    }
}

void SerialHandler::switchYard() { 
    uint8_t cmdId;

    cmdId = readInt(0); 

    switch (cmdId) {

        case constants::cmdIdResetTrigCnt:
            resetTrigCnt();
            break;

        case constants::cmdIdGetSyncSignal:
            getSyncSignal();
            break;

        case constants::cmdIdGetTrigCnt:
            getTrigCnt();
            break;

        default:
            unknownCmd();
            break;
    }
}


void SerialHandler::resetTrigCnt()
{
    systemState.resetTrigCnt();
    Serial << "[" << _DEC(constants::rspSuccess) << "]" << endl;
}

void SerialHandler::getSyncSignal()
{
    uint8_t signal;
    if (numberOfItems() < 2)
    {
        Serial << "[" << _DEC(constants::rspFail) << ",too few arguments]" << endl;
    }
    else 
    {
        unsigned long reqTrigCnt = (unsigned long)(readLong(1));
        if (systemState.getBufSyncSignal(reqTrigCnt,signal))
        {
            Serial << "[" << _DEC(constants::rspSuccess) << ",";
            for (int i=0; i<constants::numSyncSignal; i++)
            {
                uint8_t bit = 0x1 & (signal >> i);
                Serial << _DEC(bit);
                if (i < (constants::numSyncSignal-1))
                {
                    Serial << ",";
                }
            }
            Serial << "]" << endl;
        }
        else
        {
            Serial << "[" << _DEC(constants::rspFail) << ",out of range]" << endl; 
        }
    }
}

void SerialHandler::getTrigCnt()
{
    Serial << "[" << _DEC(constants::rspSuccess) << ","; 
    Serial << _DEC(systemState.getTrigCnt()) << "]" << endl;
}

void SerialHandler::unknownCmd()
{
    Serial << "[" << _DEC(constants::rspFail) << ",unknown command]" << endl;
}
