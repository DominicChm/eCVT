#include "BrakePressure.h"

BrakePressure::BrakePressure(int8_t pin) {
    this->pin = pin;
}

int16_t BrakePressure::read() {
    return analogRead(pin);
}
