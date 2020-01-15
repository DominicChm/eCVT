#include "Motor.h"

const int8_t MOT_INA = 20;
const int8_t MOT_INB = 21;
const int8_t MOT_PWM = 23;

Motor mot(MOT_INA, MOT_INB, MOT_PWM);

void setup() {
    mot.init();
}

void loop() {
    Serial.println("10");
    mot.setDutyCycle(10);
}
