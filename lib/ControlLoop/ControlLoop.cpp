#include "ControlLoop.h"


ControlLoop::ControlLoop(FSMVars fsm, PIDController pid):
   Task(fsm), pid(pid) {}


PIDController ControlLoop::getPID() {
   return pid;
}