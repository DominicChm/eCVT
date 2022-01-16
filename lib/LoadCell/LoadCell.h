#ifndef LoadCell_h
#define LoadCell_h

#include <Arduino.h>
#include "Sensor.h"
#include "HX711.h"

class LoadCell : public Sensor
{
public:
    LoadCell(int8_t sck, int8_t sda, int32_t calibration);

    void begin();
    int16_t read();

private:
    int8_t sck;
    int8_t sda;
    int32_t calibration;
    int16_t value;
    HX711 lc;
};

#endif
