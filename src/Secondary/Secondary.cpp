#include "Secondary.h"

const float SHIFTLINK_TOP = 6.0;                                             // Vertical Displacement (in)
const float SHIFTLINK_ALL = 11.0;                                            // Vertical Displacement (in)
const float SCALE_LOADCELL_TO_CLAMPING = -1 * SHIFTLINK_ALL / SHIFTLINK_TOP; // Ratio (unitless)
const float SCALE_CLAMPING_TO_LOADCELL = -1 * SHIFTLINK_TOP / SHIFTLINK_ALL; // Ratio (unitless)
const int16_t DISENGAGED_CLAMPINGFORCE = 50;                                 // Clamping Force (lb)

Secondary::Secondary(FSMVars &fsm, Encoder &enc, Motor mot, PIDController encPID, PIDController lcPID)
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
        lcPID.setSetpoint(sRatioToForce(fsm.ePIDOutput));
    }
    else
    {
        encPID.setSetpoint(sRatioToCounts(100));
        lcPID.setSetpoint(DISENGAGED_CLAMPINGFORCE);
    }

    encPID.calc(enc.read());
    lcPID.calc(fsm.sLoadCellForce * SCALE_LOADCELL_TO_CLAMPING);

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

int16_t Secondary::getClampingForce()
{
    return fsm.sLoadCellForce * SCALE_LOADCELL_TO_CLAMPING;
}

int32_t Secondary::sRatioToCounts(int16_t ratio)
{
    if (ratio < 0)
        return sLookup[0];
    if (ratio > 100)
        return sLookup[100];
    return sLookup[ratio];
}

/** TODO: Lookup Table **/
int32_t Secondary::sRatioToForce(int16_t ratio)
{
    if (ratio < 0)
        return cLookup[0];
    if (ratio > 100)
        return cLookup[100];
    return cLookup[ratio];
}
