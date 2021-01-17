#ifndef Communication_h
#define Communication_h

#include "Task.h"
#include "Engine/Engine.h"
#include "Primary/Primary.h"
#include "Secondary/Secondary.h"

class Communication: public Task {
    public:
        enum State {INITIALIZE, WRITE_START_DATA, STORE_ECVT_DATA, WRITE_ECVT_DATA};

        Communication(FSMVars fsm, Engine engine, Primary primary, Secondary secondary);

        void run();
        // void commISR();

    private:
        State state;
        int8_t numBytesWritten;
        Engine engine;
        Primary primary;
        Secondary secondary;
        // IntervalTimer commTimer;

        struct Data {
            uint32_t time;
            int8_t eState;
            int16_t eSpeed;
            int16_t ePID;
            int16_t eP;
            int16_t eI;
            int16_t eD;
            int8_t pState;
            int32_t pSet;
            int32_t pEnc;
            int16_t pPID;
            int8_t sState;
            int32_t sSet;
            int32_t sEnc;
            int16_t sPID;
        } data;

    // Interval Timer
    // const uint32_t COMM_PERIOD = 10000;      // Microseconds (us)
};

#endif
