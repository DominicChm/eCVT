/*
 *  Sensor.h - Abstract class header for sensors.
 *  Created by Shaina Bagri, January 2021.
 *  Released to Cal Poly Baja SAE. ;)
 */

#ifndef Sensor_h
#define Sensor_h

#include <Arduino.h>

class Sensor {
    public:
        Sensor();

        virtual int16_t read() = 0;
};

#endif
