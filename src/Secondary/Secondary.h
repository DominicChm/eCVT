#ifndef Secondary_h
#define Secondary_h

#include "Clutch/Clutch.h"
#include "PIDController.h"

class Secondary : public Clutch
{
public:
    Secondary(
        FSMVars fsm,
        PIDController encController,
        PIDController lcController,
        Encoder enc,
        Motor mot);

    bool getCalc();
    void initializeController();
    void updateController();

private:
    PIDController encPID;
    PIDController lcPID;

    int16_t readLoadCell();

    static int32_t sRatioToCounts(int16_t ratio);
    static int32_t sRatioToForce(int16_t ratio);
};

#endif
