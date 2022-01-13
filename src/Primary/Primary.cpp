#include "Primary.h"

Primary::Primary(FSMVars fsm, PIDController pid, Encoder enc, LoadCell lc, Motor mot)
    : Clutch(fsm, enc, lc, mot), pid(pid){};

bool Primary::getCalc()
{
    return fsm.pCalc;
}

void Primary::initializeController()
{
    pid.setSetpoint(0);
    pid.setLoSat(-100);
    pid.setHiSat(100);
    pid.reset();
}

void Primary::updateController()
{
    pid.setSetpoint(fsm.pSetpoint);
    pid.calc(enc.read());

    fsm.pPIDOutput = pid.get();
    if (fsm.eSpeed == 0)
    {
        mot.setDutyCycle(min(MAX_STATIC_DUTYCYCLE, fsm.pPIDOutput));
    }
    else
    {
        mot.setDutyCycle(fsm.pPIDOutput);
    }

    fsm.pCalc = false;
}
