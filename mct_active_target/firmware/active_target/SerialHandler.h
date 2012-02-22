#ifndef _SERIAL_HANDLER_H_
#define _SERIAL_HANDLER_H_
#include "SerialReceiver.h"

class SerialHandler: public SerialReceiver {
    public:
        void processMsg();

    private:
        void switchYard();

};

#endif
