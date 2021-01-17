#include "Primary.h"


Primary::Primary(FSMVars fsm, PIDController pid, Encoder enc, Motor mot):
   Clutch(fsm, pid, enc, mot) {};


int16_t Primary::getClutchSpeed() {
   return fsm.eSpeed;
}


bool Primary::getCalc() {
   return fsm.pCalc;
}


void Primary::resetCalc() {
   fsm.pCalc = false;
}


int32_t Primary::getSetpoint() {
   return fsm.pSetpoint;
}


void Primary::setPIDOutput(int16_t pid) {
   fsm.pPIDOutput = pid;
}


int16_t Primary::getPIDOutput() {
   return fsm.pPIDOutput;
}
