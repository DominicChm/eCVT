#ifndef Secondary_h
#define Secondary_h

#include "Clutch/Clutch.h"
#include "PIDController.h"

class Secondary : public Clutch
{
public:
    Secondary(
        FSMVars &fsm,
        Encoder &enc,
        Motor mot,
        PIDController encController,
        PIDController lcController);

private:
    const float SHIFTLINK_TOP = 6.0;                                                    // Vertical Displacement (in)
    const float SHIFTLINK_ALL = 11.0;                                                   // Vertical Displacement (in)
    const float SCALE_CLAMPING_TO_LOADCELL = SHIFTLINK_TOP / SHIFTLINK_ALL;             // Ratio (unitless)
    const int16_t MAX_LOADCELL_FORCE = MAX_CLAMPING_FORCE * SCALE_CLAMPING_TO_LOADCELL; // Load Cell Force (lb)
    const int16_t DISENGAGED_CLAMPINGFORCE = 50;                                        // Clamping Force (lb)

    PIDController encPID;
    PIDController lcPID;

    bool getCalc();
    void initController();
    void updateController();
    bool isSafe();

    static int32_t sRatioToCounts(int16_t ratio);
    static int32_t sRatioToForce(int16_t ratio);
};

#endif
