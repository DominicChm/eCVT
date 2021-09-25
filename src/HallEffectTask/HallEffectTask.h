#ifndef HallEffectTask_h
#define HallEffectTask_h

#include "./Task/Task.h"
#include "EngineSpeed.h"
#include "WheelSpeed.h"

class HallEffectTask : public Task
{
public:
    enum State
    {
        INITIALIZE,
        UPDATE
    };

    HallEffectTask(
        FSMVars fsm,
        EngineSpeed engineSpeed,
        WheelSpeed rWheelsSpeed);

    void run();
    void engineSpeedISR();
    void rWheelsSpeedISR();

private:
    State state;
    EngineSpeed engineSpeed;
    WheelSpeed rWheelsSpeed;
};

#endif
