#ifndef CONSTANTS_H
#define CONSTANTS_H
#include "Arduino.h"

const int TRIGGER_PIN = A0;
const long DEFAULT_TRIGGER_PERIOD = 33333;    // us
const long MINIMUM_TRIGGER_PERIOD = 1000;     // us
const long MAXIMUM_TRIGGER_PERIOD = 1000000;  // us
const unsigned int TRIGGER_HIGH_TIME = 200;    // us
const long BAUDRATE = 115200;
const int BUFFER_SIZE = 50;

#endif
