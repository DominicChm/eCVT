#ifndef Engine_h
#define Engine_h

#include <Arduino.h>
#include "ControlLoop.h"
#include "LookupTables.h"

class Engine: public ControlLoop {
    public:
        enum State {INITIALIZE, DISENGAGED, ENGAGED_REST, ENGAGED_UPDATEPID};

        Engine(FSMVars fsm, PIDController pid);

        void run();
        int8_t getState();
 
    private:
        State state;
        void ctrlISR();
        int32_t pRatioToCounts(int16_t ratio);
        int32_t sRatioToCounts(int16_t ratio);
};

#endif
