#ifndef LaunchControl_h
#define LaunchControl_h

#include "./Task/Task.h"

class LaunchControl : public Task
{
public:
    enum State
    {
        INITIALIZE,
        ECVT_ENABLED,
        ECVT_DISABLED
    };

    LaunchControl(FSMVars fsm, int8_t LAUNCH_BUTTON);

    void run();

private:
    State state = INITIALIZE;
    int8_t LAUNCH_BUTTON;
};

#endif
