#include "Clutch.h"

// Calibration
const uint32_t CALIB_DELAY = 10000; // Milliseconds (ms)
const int16_t CALIB_ESPEED = 2000;  // Revolutions per Minute (RPM)
const int8_t CALIB_DUTYCYCLE = 10;  // Magnitude of Duty Cycle Percent (%)

Clutch::Clutch(FSMVars fsm, Encoder enc, LoadCell lc, Motor mot)
    : Task(fsm), enc(enc), lc(lc), mot(mot)
{
    calTime = 0;
}

void Clutch::run()
{
    switch (state)
    {
    case INITIALIZE:
        mot.begin();
        mot.setDutyCycle(0);

        initializeController();

        state = CALIBRATE_OPEN_SHEAVES;
        return;

    case CALIBRATE_OPEN_SHEAVES:
        mot.setDutyCycle(-CALIB_DUTYCYCLE);
        calTime = millis();

        state = CALIBRATE_ZERO_ENCODER;
        return;

    case CALIBRATE_ZERO_ENCODER:
        if (millis() - calTime > CALIB_DELAY)
        {
            enc.write(0);
            mot.setDutyCycle(0);
            state = CALIBRATE_WAIT_USER;
        }
        return;

    case CALIBRATE_WAIT_USER:
        if (fsm.eSpeed > CALIB_ESPEED)
        {
            state = CONTROLLER_REST;
        }
        return;

    case CONTROLLER_REST:
        if (getCalc())
        {
            state = CONTROLLER_UPDATE;
        }
        return;

    case CONTROLLER_UPDATE:
        updateController();
        state = CONTROLLER_REST;
        return;
    }
}

int8_t Clutch::getState()
{
    return (int8_t)state;
}

Encoder Clutch::getEnc()
{
    return enc;
}

LoadCell Clutch::getLC()
{
    return lc;
}
