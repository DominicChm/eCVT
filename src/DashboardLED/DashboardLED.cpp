#include "DashboardLED.h"

const uint32_t FLASH_PERIOD = 500;      // Milliseconds (ms)


DashboardLED::DashboardLED(FSMVars fsm, int8_t UPSHIFT_LED, int8_t BKSHIFT_LED): Task(fsm) {
    prevRatio = 0;
    currRatio = fsm.ePIDOutput;
    prevTime = 0;
    this->UPSHIFT_LED = UPSHIFT_LED;
    this->BKSHIFT_LED = BKSHIFT_LED;
}


void DashboardLED::run() {
    prevRatio = currRatio;
    currRatio = fsm.ePIDOutput;

    switch(state) {
        case INITIALIZE:
            state = HUB_STATE;
            return;

        case HUB_STATE:
            if(!fsm.run) {
                prevTime = millis();
                state = FLASH_BKSHIFT;
            } else if(currRatio > prevRatio) {
                state = BKSHIFT_ON;
            } else if(currRatio < prevRatio) {
                state = UPSHIFT_ON;
            } else {
                state = BOTH_LEDS_ON;
            }
            return;

        case BKSHIFT_ON:
            digitalWrite(BKSHIFT_LED, HIGH);
            digitalWrite(UPSHIFT_LED, LOW);
            state = HUB_STATE;
            return;

        case UPSHIFT_ON:
            digitalWrite(BKSHIFT_LED, LOW);
            digitalWrite(UPSHIFT_LED, HIGH);
            state = HUB_STATE;
            return;

        case BOTH_LEDS_ON:
            digitalWrite(BKSHIFT_LED, HIGH);
            digitalWrite(UPSHIFT_LED, HIGH);
            state = HUB_STATE;
            return;
        
        case FLASH_BKSHIFT:
            digitalWrite(BKSHIFT_LED, HIGH);
            digitalWrite(UPSHIFT_LED, LOW);
            if(fsm.run) {
                state = HUB_STATE;
            } else if(millis() - prevTime > FLASH_PERIOD) {
                prevTime += FLASH_PERIOD;
                state = FLASH_UPSHIFT;
            }
            return;

        case FLASH_UPSHIFT:
            digitalWrite(BKSHIFT_LED, LOW);
            digitalWrite(UPSHIFT_LED, HIGH);
            if(fsm.run) {
                state = HUB_STATE;
            } else if(millis() - prevTime > FLASH_PERIOD) {
                prevTime += FLASH_PERIOD;
                state = FLASH_BKSHIFT;
            }
            return;
    }
}