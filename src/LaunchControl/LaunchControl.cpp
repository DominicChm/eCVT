#include "LaunchControl.h"

const int16_t LC_BRKPRESSURE    = 1640; // 13-but /adc (1640/8191 ~= 1/5)
const int16_t LC_ENGINESPEED_LO = 2000; // Revolutions per Minute (RPM)
const int16_t LC_ENGINESPEED_HI = 3000; // Revolutions per Minute (RPM)


LaunchControl::LaunchControl(FSMVars fsm, int8_t LAUNCH_BUTTON, int8_t FBRAKE_PRESSURE, int8_t RBRAKE_PRESSURE): Task(fsm) {
    this->LAUNCH_BUTTON = LAUNCH_BUTTON;
    this->FBRAKE_PRESSURE = FBRAKE_PRESSURE;
    this->RBRAKE_PRESSURE = RBRAKE_PRESSURE;
}


void LaunchControl::run() {
    switch(state) {
        case INITIALIZE:
            state = ECVT_ENABLED;
            return;
        
        case ECVT_ENABLED:
            if(!digitalRead(LAUNCH_BUTTON) &&
                analogRead(FBRAKE_PRESSURE) > LC_BRKPRESSURE &&
                analogRead(RBRAKE_PRESSURE) > LC_BRKPRESSURE &&
                fsm.eSpeed < LC_ENGINESPEED_LO) {
                fsm.run = false;
                state = ECVT_DISABLED;
            }
            return;
        
        case ECVT_DISABLED:
            if(digitalRead(LAUNCH_BUTTON) &&
                analogRead(FBRAKE_PRESSURE) < LC_BRKPRESSURE &&
                analogRead(RBRAKE_PRESSURE) < LC_BRKPRESSURE &&
                fsm.eSpeed > LC_ENGINESPEED_HI) {
                fsm.run = true;
                state = ECVT_ENABLED;
            }
            return;
    }
}