void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
}

void loop() {
    static uint8_t  var1 = 0;
    static uint16_t var2 = 0;
    static uint32_t var3 = 0;

    var1++;
    var2++;
    var3++;

    Serial1.write(1);
    Serial1.write(var1);
    Serial1.write(2);
    Serial1.write(var2);
    Serial1.write(3);
    Serial1.write(var3);

    delay(200);
}
