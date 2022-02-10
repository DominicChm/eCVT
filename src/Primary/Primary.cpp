#include "Primary.h"

const float SHIFTLINK_TOP = 6.0;                                        // Vertical Displacement (in)
const float SHIFTLINK_ALL = 11.0;                                       // Vertical Displacement (in)
const float SCALE_LOADCELL_TO_CLAMPING = SHIFTLINK_ALL / SHIFTLINK_TOP; // Ratio (unitless)
const float SCALE_CLAMPING_TO_LOADCELL = SHIFTLINK_TOP / SHIFTLINK_ALL; // Ratio (unitless)

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

int16_t Primary::getClampingForce()
{
    return fsm.pLoadCellForce * SCALE_LOADCELL_TO_CLAMPING;
}

int32_t Primary::pRatioToCounts(int16_t ratio)
{
    if (ratio < 0)
    {
        return pLookup[0];
    }
    if (ratio > 100)
    {
        return pLookup[100];
    }
    return pLookup[ratio];
}
