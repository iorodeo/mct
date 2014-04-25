#include "message_handler.h"
#include <cstring>
#include <TimerOne.h>
#include <Streaming.h>

MessageHandler::MessageHandler() {}

void MessageHandler::initialize()
{
    Serial.begin(BAUDRATE);
}

void MessageHandler::update()
{
    char buffer[BUFFER_SIZE];
    memset(buffer,0,sizeof(buffer));

    uint8_t numBytes = Serial.readBytesUntil('\n', buffer, sizeof(buffer));
    if (numBytes > 0)
    {
        String message(buffer);
        if (message.startsWith("start"))
        {
            String periodStr = message.substring(6);
            long newPeriod = 100*periodStr.toInt();
            newPeriod = max(newPeriod,MINIMUM_TRIGGER_PERIOD);
            newPeriod = min(newPeriod,MAXIMUM_TRIGGER_PERIOD);
            Timer1.stop();
            Timer1.setPeriod(newPeriod);
            Timer1.start();
        }
        else if (message.startsWith("stop"))
        {
            Timer1.stop();
        }

    }
}


