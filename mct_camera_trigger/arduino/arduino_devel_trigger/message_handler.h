#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <Arduino.h>
#include "constants.h"

class MessageHandler 
{
    public:
        MessageHandler();
        void initialize();
        void update();
};

#endif
