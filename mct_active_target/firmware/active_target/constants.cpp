#include "constants.h"

const long baudrate = 9600;

// Timer and pwm 
const long timerPeriod = 20;
const uint8_t timerCountMax = 50;

// Led Array data
const uint8_t ledArrayN = LED_ARRAY_N; 
const uint8_t ledArrayM = LED_ARRAY_M;
const uint8_t ledArray[LED_ARRAY_N][LED_ARRAY_M] = 
{ 
    { 51, 28, 13, 43, 16, 36,  6, },
    {  8, 49, 26, 23, 40,  2, 33, },
    { 31, 10, 47, 44, 17, 37,  5, },
    { 52, 29, 12, 24, 41, 14, 34, },
    {  7, 50, 27, 45, 18, 38,  4, },
    { 32,  9, 48, 25, 42, 15, 35, },
    { 53, 30, 11, 46, 22, 39,  3  }
};


// Operating modes
const uint8_t modeOff = 0;
const uint8_t modeSingleLed = 1;
const uint8_t modePattern = 2;
const uint8_t modeAll = 3;
