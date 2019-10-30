/*
 *	Motor.h - Library header for motors (adapted from DualVNH5019MotorShield library).
 *	Created by Rahul Goyal, July 1 2019.
 *	Released to Cal Poly Baja SAE. ;)
 */

#ifndef Motor_h
#define Motor_h

#include <Arduino.h>

class Motor {

	public:
		// Constructor
		Motor(uint8_t INA, uint8_t INB, uint8_t PWM);

		// Methods
		void init();
		void setDutyCycle(int8_t dutyCycle);

	private:
		uint8_t INA;
		uint8_t INB;
		uint8_t PWM;
		bool negative;
};

#endif