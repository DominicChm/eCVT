#include "LoadCell.h"

LoadCell::LoadCell(int8_t sck, int8_t sda, int32_t calibration)
{
    this->sck = sck;
    this->sda = sda;
    this->calibration = calibration;
    lc = HX711();
}

void LoadCell::begin()
{
    lc.begin(sda, sck);
    lc.set_offset(0);
}

int16_t LoadCell::read()
{
    if (lc.is_ready())
    {
        value = lc.read() / calibration;
    }
    return value;
}
