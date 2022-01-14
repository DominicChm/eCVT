#include "Communication.h"

const int8_t ECVT_DATA_SIZE = 37;   // Bytes
const int8_t START_DATA_SIZE = 2;   // Bytes
const int8_t CHECK_DATA_SIZE = 2;   // Bytes
const int8_t START_BYTE_VAL = 0xAA; // 1010 1010

Communication::Communication(FSMVars fsm, Engine engine, Primary primary, Secondary secondary)
    : Task(fsm), engine(engine), primary(primary), secondary(secondary)
{
    numBytesWritten = 0;
}

void Communication::run()
{
    switch (state)
    {
    case INITIALIZE:
        state = WRITE_START_DATA;
        return;

    case WRITE_START_DATA:
        if (fsm.comm)
        {
            Serial.write(START_BYTE_VAL);
            numBytesWritten++;
        }
        if (numBytesWritten >= 2)
        {
            state = STORE_ECVT_DATA;
        }
        return;

    case STORE_ECVT_DATA:
        data.time = micros();
        // Engine
        data.engaged = fsm.engaged;
        data.eState = engine.getState();
        data.eSpeed = fsm.eSpeed;
        data.ePID = fsm.ePIDOutput;
        data.eP = engine.getPID().getP();
        data.eI = engine.getPID().getI();
        data.eD = engine.getPID().getD();
        // Primary
        data.pState = primary.getState();
        data.pEnc = primary.getEnc().read();
        data.pLC = primary.getLC().read();
        data.pPID = fsm.pPIDOutput;
        // Secondary
        data.sState = secondary.getState();
        data.sEnc = secondary.getEnc().read();
        data.sLC = secondary.getLC().read();
        data.sPID = fsm.sPIDOutput;

        state = WRITE_ECVT_DATA;
        return;

    case WRITE_ECVT_DATA:
        return;
    }
}
