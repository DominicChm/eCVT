#ifndef LaunchControl_h
#define LaunchControl_h

#include "./Task/Task.h"

class LaunchControl: public Task {
    public:
        enum State {INITIALIZE, ECVT_ENABLED, ECVT_DISABLED};

        LaunchControl(FSMVars fsm, int8_t LAUNCH_BUTTON, int8_t FBRAKE_PRESSURE, int8_t RBRAKE_PRESSURE);

        void run();

    private:
        State state;
        int8_t LAUNCH_BUTTON;
        int8_t FBRAKE_PRESSURE;
        int8_t RBRAKE_PRESSURE;
};

#endif
