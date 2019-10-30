#include <Encoder.h>

const uint8_t ENC_A = 31;
const uint8_t ENC_B = 32;

Encoder enc(ENC_A, ENC_B);

void setup() {
    enc.write(0);
}

void loop() {
    Serial.println(enc.read());
    delay(1);
}
