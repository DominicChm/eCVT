/*
 *	Task.h - Abstract class header for tasks.
 *	Created by Shaina Bagri, January 2021.
 *	Released to Cal Poly Baja SAE. ;)
 */

#ifndef Task_h
#define Task_h

#include <Arduino.h>
#include "FSMVars.h"

class Task {
   public:
      Task(FSMVars fsm);

      virtual void run() = 0;

   protected:
      FSMVars fsm;
};

#endif
