#ifndef ControlLoop_h
#define ControlLoop_h

#include "Task.h"
#include "PIDController.h"

class ControlLoop: public Task {
   public:
      ControlLoop(FSMVars fsm, PIDController pid);

      virtual void run() = 0;
      PIDController getPID();

   protected:
      PIDController pid;      // Ratio Percent / Revolutions per Minute (%/RPM)
};

#endif