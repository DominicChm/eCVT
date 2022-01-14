#include "Secondary.h"

const int16_t DISENGAGED_CLAMPINGFORCE = 50;                      // lb
const float SCALE_CLAMPINGFORCE_TO_LOADCELLFORCE = (float)6 / 11; // unitless

Secondary::Secondary(FSMVars fsm, PIDController encPID, PIDController lcPID, Encoder enc, LoadCell lc, Motor mot)
    : Clutch(fsm, enc, lc, mot), encPID(encPID), lcPID(lcPID){};

bool Secondary::getCalc()
{
    return fsm.sCalc;
}

void Secondary::initializeController()
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
    lcPID.calc(lc.read());

    fsm.sPIDOutput = encPID.get() + lcPID.get();
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
