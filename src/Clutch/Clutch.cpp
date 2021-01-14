#include "Clutch.h"

// Calibration
const uint32_t CALIB_DELAY = 10000;      // Milliseconds (ms)
const int16_t CALIB_ESPEED = 2000;      // Revolutions per Minute (RPM)
const int8_t CALIB_DUTYCYCLE = 10;     // Magnitude of Duty Cycle Percent (%)

const int8_t MAX_STATIC_DUTYCYCLE = 25;  // Magnitude of Duty Cycle Percent (%)


Clutch::Clutch(FSMVars fsm, PIDController pid, Encoder enc, Motor mot):
   ControlLoop(fsm, pid), enc(enc), mot(mot) {
      calTime = 0;
}


void Clutch::run() {
   switch(state) {
      case INITIALIZE:
         mot.begin();
         mot.setDutyCycle(0);

         pid.setSetpoint(0);
         pid.setLoSat(-100);
         pid.setHiSat( 100);
         pid.reset();

         state = CALIBRATE_OPEN_SHEAVES;
         return;
      
      case CALIBRATE_OPEN_SHEAVES:
         mot.setDutyCycle(-CALIB_DUTYCYCLE);
         calTime = millis();

         state = CALIBRATE_ZERO_ENCODER;
         return;

      case CALIBRATE_ZERO_ENCODER:
         if(millis() - calTime > CALIB_DELAY) {
            enc.write(0);
            mot.setDutyCycle(0);
            state = CALIBRATE_WAIT_USER;
         }
         return;

      case CALIBRATE_WAIT_USER:
         if(fsm.eSpeed > CALIB_ESPEED) {
            state = PCONTROLLER_REST;
         }
         return;
    
      case PCONTROLLER_REST:
         if(getCalc()) {
            state = PCONTROLLER_UPDATE;
         }
         return;

      case PCONTROLLER_UPDATE:
         pid.setSetpoint(getSetpoint());
         pid.calc(enc.read());      // *** Move to ISR Function...

         setPIDOutput(pid.get());
         if(getClutchSpeed()==0) {
            mot.setDutyCycle(min(MAX_STATIC_DUTYCYCLE, getPIDOutput()));
         } else {
            mot.setDutyCycle(getPIDOutput());
         }

         resetCalc();
         state = PCONTROLLER_REST;
         return;
   }
}


int8_t Clutch::getState() {
   return (int8_t) state;
}


Encoder Clutch::getEnc() {
   return enc;
}