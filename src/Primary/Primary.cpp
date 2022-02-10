#include "Primary.h"

Primary::Primary(FSMVars &fsm, Encoder &enc, Motor mot, PIDController pid)
    : Clutch(fsm, enc, mot), pid(pid){};

bool Primary::getCalc()
{
    return fsm.pCalc;
}

void Primary::initController()
{
    pid.setSetpoint(0);
    pid.setLoSat(-100);
    pid.setHiSat(100);
    pid.reset();
}

void Primary::updateController()
{
    // Disengaged: Encoder Count = 0
    //  Low Ratio: Encoder Count = pRatioToCounts(100)
    // High Ratio: Encoder Count = pRatioToCounts(0)

    if (fsm.engaged)
    {
        pid.setSetpoint(pRatioToCounts(fsm.ePIDOutput));
    }
    else
    {
        pid.setSetpoint(0);
    }

    pid.calc(enc.read());
    fsm.pPIDOutput = pid.get();

    if (fsm.eSpeed == 0)
    {
        setMotorDutyCycle(min(MAX_STATIC_DUTYCYCLE, fsm.pPIDOutput));
    }
    else
    {
        setMotorDutyCycle(fsm.pPIDOutput);
    }

    fsm.pCalc = false;
}

bool Primary::isSafe()
{
    return fsm.pLoadCellForce < MAX_LOADCELL_FORCE;
}

int32_t Primary::pRatioToCounts(int16_t ratio)
{
    if (ratio < 0)
    {
        return pLookup[0];
    }
    else if (ratio > 100)
    {
        return pLookup[100];
    }
    return pLookup[ratio];
}
