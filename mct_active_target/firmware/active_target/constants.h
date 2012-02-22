#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

const long baudrate = 9600;

// Timer and pwm 
const long timerPeriod = 20;
const uint8_t timerCountMax = 50;

// Led Array data
const uint8_t ledArrayN = 7; 
const uint8_t ledArrayM = 7;
const uint8_t ledArray[ledArrayN][ledArrayM] = 
{ 
    { 51, 28, 13, 43, 16, 36,  6, },
    {  8, 49, 26, 23, 40,  2, 33, },
    { 31, 10, 47, 44, 17, 37,  5, },
    { 52, 29, 12, 24, 41, 14, 34, },
    {  7, 50, 27, 45, 18, 38,  4, },
    { 32,  9, 48, 25, 42, 15, 35, },
    { 53, 30, 11, 46, 22, 39,  3  }
};


// Serial Comand ids
const uint8_t cmdOff = 0;
const uint8_t cmdSingleLed = 1;
const uint8_t cmdPattern = 2;

// Operating modes
const uint8_t modeOff = 0;
const uint8_t modeSingleLed = 1;
const uint8_t modePattern = 2;

#endif

