#ifndef FSMVars_h
#define FSMVars_h

#include <Arduino.h>

class FSMVars {
   public:
      FSMVars();

      volatile bool run;
      volatile bool eCalc;
      volatile bool pCalc;
      volatile bool sCalc;
      volatile bool comm;
      int16_t eSpeed;
      int16_t rwSpeed;
      int32_t pSetpoint;
      int32_t sSetpoint;
      int16_t ePIDOutput;
      int16_t pPIDOutput;
      int16_t sPIDOutput;

};

#endif