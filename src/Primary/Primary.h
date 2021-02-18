#ifndef Primary_h
#define Primary_h

#include "Clutch/Clutch.h"

class Primary: public Clutch {
    public:
        Primary(
            FSMVars fsm,
            PIDController pid,
            Encoder enc,
            Motor mot);

        int16_t getClutchSpeed();
        bool getCalc();
        void resetCalc();
        int32_t getSetpoint();
        void setPIDOutput(int16_t pid);
        int16_t getPIDOutput();
};

#endif
