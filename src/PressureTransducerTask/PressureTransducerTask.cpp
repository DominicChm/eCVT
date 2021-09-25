#include "PressureTransducerTask.h"

PressureTransducerTask::PressureTransducerTask(
    FSMVars fsm, BrakePressure fBrakePressure, BrakePressure rBrakePressure) : Task(fsm), fBrakePressure(fBrakePressure), rBrakePressure(rBrakePressure) {}

void PressureTransducerTask::run()
{
    switch (state)
    {
    case INITIALIZE:
        state = UPDATE;
        return;

    case UPDATE:
        fsm.fBrakePressure = fBrakePressure.read();
        fsm.rBrakePressure = rBrakePressure.read();
        return;
    }
}
