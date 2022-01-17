#ifndef LoadCellTask_h
#define LoadCellTask_h

#include "./Task/Task.h"
#include "LoadCell.h"

class LoadCellTask : public Task
{
public:
    enum State
    {
        INITIALIZE,
        UPDATE
    };

    LoadCellTask(
        FSMVars fsm,
        LoadCell pLoadCell,
        LoadCell sLoadCell);

    void run();
    void engineSpeedISR();
    void rWheelsSpeedISR();

private:
    State state = INITIALIZE;
    LoadCell pLoadCell;
    LoadCell sLoadCell;
};

#endif
