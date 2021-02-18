#ifndef Clutch_h
#define Clutch_h

#include "./ControlLoop/ControlLoop.h"
#include "Motor.h"
#include <Encoder.h>

class Clutch: public ControlLoop {
    public:
        enum State {
            INITIALIZE,
            CALIBRATE_OPEN_SHEAVES,
            CALIBRATE_ZERO_ENCODER,
            CALIBRATE_WAIT_USER,
            PCONTROLLER_REST,
            PCONTROLLER_UPDATE};

        Clutch(
            FSMVars fsm,
            PIDController pid,
            Encoder enc,
            Motor mot);

        void run();
        Encoder getEnc();
        int8_t getState();

        virtual int16_t getClutchSpeed() = 0;
        virtual bool getCalc() = 0;
        virtual void resetCalc() = 0;
        virtual int32_t getSetpoint() = 0;
        virtual void setPIDOutput(int16_t pid) = 0;
        virtual int16_t getPIDOutput() = 0;
      
    protected:
        State state;
        Encoder enc;
        Motor mot;
        uint32_t calTime;       // Milliseconds (ms)
};

#endif
