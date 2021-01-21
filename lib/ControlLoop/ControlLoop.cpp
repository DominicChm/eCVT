/*
 *	ControlLoop.cpp - Abstract class for control loops.
 *	Created by Shaina Bagri, January 2021.
 *	Released to Cal Poly Baja SAE. ;)
 */

#include "ControlLoop.h"


ControlLoop::ControlLoop(FSMVars fsm, PIDController pid):
   Task(fsm), pid(pid) {}


PIDController ControlLoop::getPID() {
   return pid;
}
