#include <Encoder.h>

const int8_t ENC_A = 24;
const int8_t ENC_B = 25;

Encoder enc(ENC_A, ENC_B);
                                                                                        
void setup() {
    enc.write(0);
}

void loop() {
    Serial.println(enc.read());
    delay(1000);
}
