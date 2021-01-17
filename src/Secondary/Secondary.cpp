#include "Secondary.h"


Secondary::Secondary(FSMVars fsm, PIDController pid, Encoder enc, Motor mot):
   Clutch(fsm, pid, enc, mot) {};


int16_t Secondary::getClutchSpeed() {
   return fsm.rwSpeed;
}


bool Secondary::getCalc() {
   return fsm.sCalc;
}


void Secondary::resetCalc() {
   fsm.sCalc = false;
}


int32_t Secondary::getSetpoint() {
   return fsm.sSetpoint;
}


void Secondary::setPIDOutput(int16_t pid) {
   fsm.sPIDOutput = pid;
}


int16_t Secondary::getPIDOutput() {
   return fsm.sPIDOutput;
}
