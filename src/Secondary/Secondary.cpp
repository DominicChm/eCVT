#include "Secondary.h"

Secondary::Secondary(FSMVars fsm, PIDController pid, Encoder enc, Motor mot) : Clutch(fsm, enc, mot), pid(pid){};

bool Secondary::getCalc()
{
    return fsm.sCalc;
}

void Secondary::initializeController()
{
    pid.setSetpoint(0);
    pid.setLoSat(-100);
    pid.setHiSat(100);
    pid.reset();
}

void Secondary::updateController()
{
    pid.setSetpoint(fsm.sSetpoint);
    pid.calc(enc.read());

    fsm.sPIDOutput = pid.get();
    if (fsm.rwSpeed == 0)
    {
        mot.setDutyCycle(min(MAX_STATIC_DUTYCYCLE, fsm.sPIDOutput));
    }
    else
    {
        mot.setDutyCycle(fsm.sPIDOutput);
    }

    fsm.sCalc = false;
}
