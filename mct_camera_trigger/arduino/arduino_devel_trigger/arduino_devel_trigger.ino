#include <TimerOne.h>
#include <Streaming.h>
#include "constants.h"
#include "message_handler.h"

void onTimer();

MessageHandler msgHandler;

void setup()
{
    msgHandler.initialize();

    pinMode(TRIGGER_PIN,OUTPUT);
    digitalWrite(TRIGGER_PIN,LOW);
    Timer1.initialize(DEFAULT_TRIGGER_PERIOD);
    Timer1.attachInterrupt(onTimer);
}

void loop()
{
    msgHandler.update();
}

void onTimer()
{
    interrupts(); // allow interrutps
    digitalWrite(TRIGGER_PIN,HIGH);
    delayMicroseconds(TRIGGER_HIGH_TIME);
    digitalWrite(TRIGGER_PIN,LOW);
}


