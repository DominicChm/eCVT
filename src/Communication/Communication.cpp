#include "Communication.h"

#define COMM_SERIAL Serial3

const int8_t ECVT_DATA_SIZE = sizeof(Communication::Data);  // Bytes
const int8_t START_DATA_SIZE = 2;   // Bytes
const int8_t CHECK_DATA_SIZE = 2;   // Bytes
const int8_t START_BYTE_VAL = 0xAA; // 1010 1010

Communication::Communication(FSMVars &fsm, Engine engine, Primary primary, Secondary secondary)
        : Task(fsm), engine(engine), primary(primary), secondary(secondary) {
    numBytesWritten = 0;
}

void Communication::run() {
    switch (state) {
        case INITIALIZE:
            state = INIT_WRITE;
            COMM_SERIAL.begin(9600);
            return;

        case INIT_WRITE:
            numBytesWritten = 0;
            state = WRITE_START_DATA;
            return;

        case WRITE_START_DATA:
            if (fsm.comm) {
                COMM_SERIAL.write(START_BYTE_VAL);
                numBytesWritten++;
            }
            if (numBytesWritten >= 2) {
                fsm.comm = false;
                //Serial.println("WRT");
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
            data.pLC = fsm.pLoadCellForce;
            data.pPID = fsm.pPIDOutput;
            // Secondary
            data.sState = secondary.getState();
            data.sEnc = secondary.getEnc().read();
            data.sLC = fsm.sLoadCellForce;
            data.sPID = fsm.sPIDOutput;

            COMM_SERIAL.write((uint8_t * ) &data, ECVT_DATA_SIZE);
            state = WRITE_ECVT_DATA;
            return;

        case WRITE_ECVT_DATA:
            //Checksum;
            COMM_SERIAL.write(0xFF);
            COMM_SERIAL.write(0xFF);
            state = INIT_WRITE;
            return;
    }
}
