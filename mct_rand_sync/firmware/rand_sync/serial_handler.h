#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H
#include <SerialReceiver.h>
#include "system_state.h"

class SerialHandler: public SerialReceiver 
{
    public:
        SerialHandler();
        void processInput();
    private:
        void switchYard();
        void resetTrigCnt();
        void getSyncSignal();
        void getTrigCnt();
        void unknownCmd();
};

extern SystemState systemState;

#endif
