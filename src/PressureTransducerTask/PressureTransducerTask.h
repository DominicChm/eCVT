#ifndef PressureTransducer_h
#define PressureTransducer_h

#include "./Task/Task.h"
#include "BrakePressure.h"

class PressureTransducerTask : public Task
{
public:
    enum State
    {
        INITIALIZE,
        UPDATE
    };

    PressureTransducerTask(
        FSMVars &fsm,
        BrakePressure fBrakePressure,
        BrakePressure rBrakePressure);

    void run();

private:
    State state = INITIALIZE;
    BrakePressure fBrakePressure;
    BrakePressure rBrakePressure;
};

#endif
