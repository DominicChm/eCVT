#include "Secondary.h"

const float SCALE_CLAMPINGFORCE_TO_LOADCELLFORCE = (float)6 / 11;

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
    encPID.setSetpoint(fsm.sSetpoint);
    encPID.calc(enc.read());
    lcPID.setSetpoint(fsm.cSetpoint * SCALE_CLAMPINGFORCE_TO_LOADCELLFORCE);
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
