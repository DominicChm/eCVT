#ifndef Secondary_h
#define Secondary_h

#include "Clutch.h"

class Secondary: public Clutch {
   public:
      Secondary(FSMVars fsm, PIDController pid, Encoder enc, Motor mot);

      int16_t getClutchSpeed();
      bool getCalc();
      void resetCalc();
      int32_t getSetpoint();
      void setPIDOutput(int16_t pid);
      int16_t getPIDOutput();
};

#endif