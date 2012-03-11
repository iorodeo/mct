#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Led array size parameters
enum {LED_ARRAY_N=7};
enum {LED_ARRAY_M=7};

// Serial Comand IDs
enum {
    CMD_OFF        = 0,
    CMD_SINGLE_LED = 1,
    CMD_PATTERN    = 2,
    CMD_ALL        = 3,
};

extern const long baudrate;

// Timer and pwm 
extern const long timerPeriod;
extern const uint8_t timerCountMax;

// Led Array data
extern const uint8_t ledArrayN; 
extern const uint8_t ledArrayM; 
extern const uint8_t ledArray[LED_ARRAY_N][LED_ARRAY_M];

// Operating modes
extern const uint8_t modeOff;
extern const uint8_t modeSingleLed;
extern const uint8_t modePattern;
extern const uint8_t modeAll;

#endif

