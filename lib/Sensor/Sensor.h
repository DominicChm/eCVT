#ifndef Sensor_h
#define Sensor_h

#include "FSMVars.h"

class Sensor {
    public:
        Sensor(FSMVars fsm);

        virtual void read() = 0;

    protected:
        FSMVars fsm;
};

#endif