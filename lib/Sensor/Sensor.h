#ifndef Sensor_h
#define Sensor_h

#include "FSMVars.h"

class Sensor {
    public:
        Sensor(FSMVars fsm);

        virtual int16_t read() = 0;

    protected:
        FSMVars fsm;
};

#endif