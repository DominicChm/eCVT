#include "Engine.h"

const int16_t ENGAGE_SPEED = 2400; // Revolutions per Minute (RPM)
const int16_t SHIFT_SPEED = 3200;  // Revolutions per Minute (RPM)

// TODO: UPDATE DOCUMENTATION (load cell replaces sheave offset)
// eCVT Sheave Offset
/** This constant accounts for mechanical imperfections and adjusts belt clamping force.
    Examples of mechanical imperfections include:
       1. belt wear
       2. deflection
       3. manufacturing tolerances
    This constant offsets the ideal sheave position (as determined by the lookup table) by a 
    number of encoder counts. A larger number indicates increased clamping and a smaller number
    indicates decreased clamping. The effective change in clamping is determined by
    P * SHEAVE_OFFSET = VOLTS, where P is the proportional gain for the respective clutch and
    VOLTS is the voltage applied to the motor at the ideal sheave position. **/
const int32_t SHEAVE_OFFSET = 0; // Encoder Counts (~1/3606 of a revolution)

Engine::Engine(FSMVars &fsm, PIDController pid) : ControlLoop(fsm, pid) {}

void Engine::run()
{
    switch (state)
    {
    case INITIALIZE:
        pid.setSetpoint(SHIFT_SPEED);
        pid.setLoSat(0);
        pid.setHiSat(100);
        pid.reset();

        state = DISENGAGED;
        return;

    case DISENGAGED:
        fsm.engaged = false;
        if (fsm.eSpeed > ENGAGE_SPEED && fsm.run)
        {
            pid.reset();
            fsm.engaged = true;
            state = ENGAGED_REST;
        }
        return;

    case ENGAGED_REST:
        if (fsm.eSpeed < ENGAGE_SPEED || !fsm.run)
        {
            state = DISENGAGED;
        }
        else if (fsm.eCalc)
        {
            state = ENGAGED_UPDATEPID;
        }
        return;

    case ENGAGED_UPDATEPID:
        pid.calc(fsm.eSpeed);
        fsm.ePIDOutput = pid.get();

        fsm.eCalc = false;
        state = ENGAGED_REST;
        return;
    }
}

int8_t Engine::getState()
{
    return (int8_t)state;
}
