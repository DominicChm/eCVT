#include "Clutch.h"

// Calibration
const int8_t CALIB_DUTYCYCLE = 10;    // Magnitude of Duty Cycle Percent (%)
const int16_t CALIB_FORCE = 50;       // Clamping Force (lb)
const int16_t CALIB_ESPEED = 2000;    // Revolutions per Minute (RPM)
const uint32_t CALIB_MINTIME = 250;   // Milliseconds (ms)
const uint32_t CALIB_MAXTIME = 10000; // Milliseconds (ms)

Clutch::Clutch(FSMVars &fsm, Encoder &enc, Motor mot)
    : Task(fsm), enc(enc), mot(mot)
{
    calTime = 0;
}

void Clutch::run()
{
    switch (state)
    {
    case INITIALIZE:
        mot.begin();
        setMotorDutyCycle(0);

        initController();

        state = CALIBRATE_OPEN_SHEAVES;
        return;

    case CALIBRATE_OPEN_SHEAVES:
        setMotorDutyCycle(-CALIB_DUTYCYCLE);
        calTime = millis();

        state = CALIBRATE_ZERO_ENCODER;
        return;

    case CALIBRATE_ZERO_ENCODER:
        if (millis() - calTime < CALIB_MINTIME)
        {
            return;
        }

        // Note: "clamping" force is negative when opening sheaves
        if (-getClampingForce() > CALIB_FORCE)
        {
            enc.write(0);
            setMotorDutyCycle(0);
            state = CALIBRATE_WAIT_USER;
        }

        if (millis() - calTime > CALIB_MAXTIME)
        {
            state = ERROR;
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

    case ERROR:
        setMotorDutyCycle(0);
        return;
    }
}

void Clutch::setMotorDutyCycle(int16_t dutyCycle)
{
    int16_t force = getClampingForce();

    // Do not use Arduino's abs() function - it is a poorly written macro
    // https://www.arduino.cc/reference/en/language/functions/math/abs/
    force = force < 0 ? -force : force;

    if (force > MAX_CLAMPING_FORCE)
    {
        dutyCycle = 0;
    }
    mot.setDutyCycle(dutyCycle);
}

int8_t Clutch::getState()
{
    return (int8_t)state;
}

Encoder Clutch::getEnc()
{
    return enc;
}
