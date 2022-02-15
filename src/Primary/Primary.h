#ifndef Primary_h
#define Primary_h

#include "Clutch/Clutch.h"
#include "PIDController.h"

class Primary : public Clutch
{
public:
    Primary(
        FSMVars &fsm,
        Encoder &enc,
        Motor mot,
        PIDController pid);

private:
    PIDController pid;

    bool getCalc();
    void initController();
    void updateController();
    int16_t getClampingForce();

    static int32_t pRatioToCounts(int16_t ratio);
};

#endif
