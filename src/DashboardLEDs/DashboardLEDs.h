#ifndef DashboardLED_h
#define DashboardLED_h

#include "./Task/Task.h"

class DashboardLEDs: public Task {
    public:
        enum State {INITIALIZE, HUB_STATE, BKSHIFT_ON, UPSHIFT_ON, BOTH_LEDS_ON, FLASH_BKSHIFT, FLASH_UPSHIFT};

        DashboardLEDs(FSMVars fsm, int8_t UPSHIFT_LED, int8_t BKSHIFT_LED);

        void run();
    
    private:
        State state;
        int16_t prevRatio;      // Ratio Percent (%)
        int16_t currRatio;      // Ratio Percent (%)
        uint32_t prevTime;      // Milliseconds (ms)
        int8_t UPSHIFT_LED;
        int8_t BKSHIFT_LED;
};

#endif
