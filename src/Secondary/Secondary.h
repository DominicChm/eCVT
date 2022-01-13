#ifndef Secondary_h
#define Secondary_h

#include "Clutch/Clutch.h"
#include "PIDController.h"

class Secondary : public Clutch
{
public:
    Secondary(
        FSMVars fsm,
        PIDController pid,
        Encoder enc,
        Motor mot);

    bool getCalc();
    void initializeController();
    void updateController();

private:
    PIDController pid;
};

#endif
