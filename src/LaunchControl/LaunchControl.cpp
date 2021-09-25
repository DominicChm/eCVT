#include "LaunchControl.h"

const int16_t LC_BRKPRESSURE = 1640;    // 13-bit ADC (1640/8191 ~= 1/5)
const int16_t LC_ENGINESPEED_LO = 2000; // Revolutions per Minute (RPM)
const int16_t LC_ENGINESPEED_HI = 3000; // Revolutions per Minute (RPM)

LaunchControl::LaunchControl(FSMVars fsm, int8_t LAUNCH_BUTTON) : Task(fsm)
{
    this->LAUNCH_BUTTON = LAUNCH_BUTTON;
}

void LaunchControl::run()
{
    switch (state)
    {
    case INITIALIZE:
        state = ECVT_ENABLED;
        return;

    case ECVT_ENABLED:
        if (!digitalRead(LAUNCH_BUTTON) &&
            analogRead(fsm.fBrakePressure) > LC_BRKPRESSURE &&
            analogRead(fsm.rBrakePressure) > LC_BRKPRESSURE &&
            fsm.eSpeed < LC_ENGINESPEED_LO)
        {
            fsm.run = false;
            state = ECVT_DISABLED;
        }
        return;

    case ECVT_DISABLED:
        if (digitalRead(LAUNCH_BUTTON) &&
            fsm.fBrakePressure < LC_BRKPRESSURE &&
            fsm.rBrakePressure < LC_BRKPRESSURE &&
            fsm.eSpeed > LC_ENGINESPEED_HI)
        {
            fsm.run = true;
            state = ECVT_ENABLED;
        }
        return;
    }
}
