#include "Communication.h"

const int8_t   ECVT_DATA_SIZE = 37;       // Bytes
const int8_t   START_DATA_SIZE = 2;       // Bytes
const int8_t   CHECK_DATA_SIZE = 2;       // Bytes
const int8_t START_BYTE_VAL = 0xAA;       // 1010 1010


Communication::Communication(FSMVars fsm, Engine engine, Primary primary, Secondary secondary):
    Task(fsm), engine(engine), primary(primary), secondary(secondary) {
    this->numBytesWritten = 0;
}


void Communication::run() {
    switch(state) {
        case INITIALIZE:
            // commTimer.begin(commISR, COMM_PERIOD);
            state = WRITE_START_DATA;
            return;
        
        case WRITE_START_DATA:
            if(fsm.comm) {
                Serial.write(START_BYTE_VAL);
                numBytesWritten++;
            }
            if(numBytesWritten >= 2) {
                state = STORE_ECVT_DATA;
            }
            return;

        case STORE_ECVT_DATA:
            data.time = micros();
            data.eState = engine.getState();
            data.eSpeed = fsm.eSpeed;
            data.ePID = fsm.ePIDOutput;
            data.eP = engine.getPID().getP();
            data.eI = engine.getPID().getI();
            data.eD = engine.getPID().getD();
            data.pState = primary.getState();
            data.pSet = fsm.pSetpoint;
            data.pEnc = primary.getEnc().read();
            data.pPID = fsm.pPIDOutput;
            data.sState = secondary.getState();
            data.sSet = fsm.sSetpoint;
            data.sEnc = secondary.getEnc().read();
            data.sPID = fsm.sPIDOutput;
            
            state = WRITE_ECVT_DATA;
            return;

        case WRITE_ECVT_DATA:
            return; 
    }
}


// void Communication::commISR() { fsm.comm = true; }