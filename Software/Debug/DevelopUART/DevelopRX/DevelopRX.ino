void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
}

void loop() {

    static byte input;
    static int8_t num;
    static String var;
    
    if (Serial1.available()) {
        input = Serial.read();
        switch (in) {
            case 1:
                num = 1;
                var = "var1";
                return;
            case 2:
                num = 2;
                var = "var2";
                return;
            case 3;
                num = 4;
                var = "var3";
                return;
        }
        for (int i = 0; i < num; i++) {
            while (!Serial.available) { }
            
        }
    }
}
