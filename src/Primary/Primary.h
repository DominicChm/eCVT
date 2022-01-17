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

    bool getCalc();
    void initializeController();
    void updateController();

private:
    PIDController pid;

    int16_t readLoadCell();

    static int32_t pRatioToCounts(int16_t ratio);
};

#endif
