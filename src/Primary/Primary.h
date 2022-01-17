#ifndef Primary_h
#define Primary_h

#include "Clutch/Clutch.h"
#include "PIDController.h"

class Primary : public Clutch
{
public:
    Primary(
        FSMVars fsm,
        PIDController pid,
        Encoder enc,
        Motor mot);

private:
    const float SHIFTLINK_TOP = 6.0;                                                    // Vertical Displacement (in)
    const float SHIFTLINK_ALL = 11.0;                                                   // Vertical Displacement (in)
    const float SCALE_CLAMPING_TO_LOADCELL = SHIFTLINK_TOP / SHIFTLINK_ALL;             // Ratio (unitless)
    const int16_t MAX_LOADCELL_FORCE = MAX_CLAMPING_FORCE * SCALE_CLAMPING_TO_LOADCELL; // Load Cell Force (lb)

    PIDController pid;

    bool getCalc();
    void initController();
    void updateController();
    bool isSafe();

    static int32_t pRatioToCounts(int16_t ratio);
};

#endif
