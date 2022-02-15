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
    PIDController encPID;
    PIDController lcPID;

    bool getCalc();
    void initController();
    void updateController();
    int16_t getClampingForce();

    static int32_t sRatioToCounts(int16_t ratio);
    static int32_t sRatioToForce(int16_t ratio);
};

#endif
