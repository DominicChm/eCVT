#ifndef Sensor_h
#define Sensor_h

#include "FSMVars.h"

class Sensor {
    public:
        Sensor();

        virtual int16_t read() = 0;
};

#endif