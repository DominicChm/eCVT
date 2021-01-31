/*
 *	ControlLoop.h - Abstract class header for control loops.
 *	Created by Shaina Bagri, January 2021.
 *	Released to Cal Poly Baja SAE. ;)
 */

#ifndef ControlLoop_h
#define ControlLoop_h

#include "./Task/Task.h"
#include "PIDController.h"

class ControlLoop: public Task {
    public:
        ControlLoop(FSMVars fsm, PIDController pid);

        virtual void run() = 0;
        virtual int8_t getState() = 0;
        PIDController getPID();

    protected:
        PIDController pid;      // Ratio Percent / Revolutions per Minute (%/RPM)
};

#endif
