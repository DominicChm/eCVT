#ifndef LaunchControl_h
#define LaunchControl_h

#include "Task.h"
#include "WiringDec2019.h"  // NEED TO REMOVE

class LaunchControl: public Task {
    public:
        enum State {INITIALIZE, ECVT_ENABLED, ECVT_DISABLED};

        LaunchControl(FSMVars fsm);

        void run();

    private:
        State state;
};

#endif