#ifndef Task_h
#define Task_h

#include <Arduino.h>
#include "FSMVars.h"

class Task {
   public:
      Task(FSMVars fsm);

      virtual void run() = 0;
      virtual int8_t getState() = 0;

   protected:
      FSMVars fsm;
};

#endif