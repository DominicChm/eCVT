#ifndef BrakePressure_h
#define BrakePressure_h

#include <Arduino.h>
#include "Sensor.h"

class BrakePressure : public Sensor {
    public:
        BrakePressure(int8_t pin);

        int16_t read();
    
    private:
        int8_t pin;
};

#endif
