#include "Secondary.h"

const int16_t DISENGAGED_CLAMPINGFORCE = 50;                      // lb
const float SCALE_CLAMPINGFORCE_TO_LOADCELLFORCE = (float)6 / 11; // unitless

Secondary::Secondary(FSMVars fsm, PIDController encPID, PIDController lcPID, Encoder enc, Motor mot)
    : Clutch(fsm, enc, mot), encPID(encPID), lcPID(lcPID){};

bool Secondary::getCalc()
{
    return fsm.sCalc;
}

void Secondary::initController()
{
    // TODO: Saturate combined output

    // Encoder
    encPID.setSetpoint(0);
    encPID.setLoSat(-100);
    encPID.setHiSat(100);
    encPID.reset();

    // Load Cell
    lcPID.setSetpoint(0);
    lcPID.setLoSat(-100);
    lcPID.setHiSat(100);
    lcPID.reset();
}

void Secondary::updateController()
{
    // Disengaged: Encoder Count = sRatioToCounts(100)
    //  Low Ratio: Encoder Count = sRatioToCounts(100)
    // High Ratio: Encoder Count = sRatioToCounts(0)

    if (fsm.engaged)
    {
        encPID.setSetpoint(sRatioToCounts(fsm.ePIDOutput));
        lcPID.setSetpoint(sRatioToForce(fsm.ePIDOutput) * SCALE_CLAMPINGFORCE_TO_LOADCELLFORCE);
    }
    else
    {
        encPID.setSetpoint(sRatioToCounts(100));
        lcPID.setSetpoint(DISENGAGED_CLAMPINGFORCE * SCALE_CLAMPINGFORCE_TO_LOADCELLFORCE);
    }

    encPID.calc(enc.read());
    lcPID.calc(fsm.sLoadCellForce);

    fsm.sPIDOutput = encPID.get() + lcPID.get();
    if (fsm.rwSpeed == 0)
    {
        setMotorDutyCycle(min(MAX_STATIC_DUTYCYCLE, fsm.sPIDOutput));
    }
    else
    {
        setMotorDutyCycle(fsm.sPIDOutput);
    }

    fsm.sCalc = false;
}

int16_t Secondary::readLoadCell()
{
    return fsm.sLoadCellForce;
}

int32_t Secondary::sRatioToCounts(int16_t ratio)
{
    if (ratio < 0)
    {
        return sLookup[0];
    }
    else if (ratio > 100)
    {
        return sLookup[100];
    }
    return sLookup[ratio];
}

/** TODO: Lookup Table **/
int32_t Secondary::sRatioToForce(int16_t ratio)
{
    return 0;
    // if (ratio < 0)
    // {
    //     return cLookup[0];
    // }
    // else if (ratio > 100)
    // {
    //     return cLookup[100];
    // }
    // return cLookup[ratio];
}
