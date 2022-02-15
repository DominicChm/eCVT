#include "LoadCellTask.h"

LoadCellTask::LoadCellTask(FSMVars &fsm, LoadCell pLoadCell, LoadCell sLoadCell)
    : Task(fsm), pLoadCell(pLoadCell), sLoadCell(sLoadCell) {}

void LoadCellTask::run()
{
    switch (state)
    {
    case INITIALIZE:
        pLoadCell.begin();
        sLoadCell.begin();
        state = UPDATE;
        return;

    case UPDATE:
        fsm.pLoadCellForce = pLoadCell.read();
        fsm.sLoadCellForce = sLoadCell.read();
        return;
    }
}
