#include "constants.h"
#include "Streaming.h"
#include "SerialHandler.h"
#include "State.h"

extern State sysState;

void SerialHandler::processMsg() {
    while (Serial.available() > 0) {
        process(Serial.read());
        if (messageReady()) {
            switchYard();
            reset();
        }
    }
}


void SerialHandler::switchYard() {
    uint8_t cmd = readInt(0);

    switch (cmd) {
        case cmdOff:
            Serial << "cmdOff" << endl;
            sysState.setModeOff();
            break;

        case cmdSingleLed:
            Serial << "cmdSetLed" << endl;
            {
                uint8_t i;
                uint8_t j;
                uint8_t newLedPower;
                i = (uint8_t) readInt(1);
                j = (uint8_t) readInt(2);
                newLedPower = (uint8_t) readInt(3);
                Serial << "i,j: " << i << "," << j << "  led power: " << newLedPower << endl;
                sysState.setModeSingleLed(i,j,newLedPower);
            }
            break;

        case cmdPattern:
            Serial << "cmdPattern" << endl;
            sysState.setModePattern();
            break;

        default:
            // We shouldn't be here - same as off.
            Serial << "default - unknown cmd" << endl;
            break;
    }
}
