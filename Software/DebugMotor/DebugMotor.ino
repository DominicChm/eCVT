#include "Motor.h"

const uint8_t MOT_INA = 11;
const uint8_t MOT_INB = 12;
const uint8_t MOT_PWM = 7;
const uint8_t ENC_A = 31;
const uint8_t ENC_B = 32;

Motor mot(MOT_INA, MOT_INB, MOT_PWM);

void setup() {
  mot.init();
}

void loop() {
  mot.setDutyCycle(5);
}
