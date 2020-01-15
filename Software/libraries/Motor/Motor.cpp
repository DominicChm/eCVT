/*
 *	Motor.cpp - Library for motors (adapted from DualVNH5019MotorShield library).
 *	Created by Rahul Goyal, July 1 2019.
 *	Released to Cal Poly Baja SAE. ;)
 */

#include <Arduino.h>
#include "Motor.h"

// Constructor
Motor::Motor(int8_t INA, int8_t INB, int8_t PWM) {
	this->INA = INA;
	this->INB = INB;
	this->PWM = PWM;
}

void Motor::init() {
	pinMode(INA, OUTPUT);
	pinMode(INB, OUTPUT);
	pinMode(PWM, OUTPUT);
}

void Motor::setDutyCycle(int8_t dutyCycle) {
	// Normalize duty cycle, determine direction
	if (dutyCycle < 0) {
		dutyCycle *= -1;
		negative = true;
	} else {
		negative = false;
	}

	// Saturate duty cycle
	if (dutyCycle > 100) {
		dutyCycle = 100;
	}

	// Write PWM, scale 100 to 255
	analogWrite(PWM, dutyCycle * 255 / 100);

	// Coast (indpendent of direction)
	if (dutyCycle == 0) {
		digitalWrite(INA, LOW);
		digitalWrite(INB, LOW);
	// Negative
	} else if (negative) {
		digitalWrite(INA, LOW);
		digitalWrite(INB, HIGH);
	// Positive
	} else {
		digitalWrite(INA, HIGH);
		digitalWrite(INB, LOW);
	}
}