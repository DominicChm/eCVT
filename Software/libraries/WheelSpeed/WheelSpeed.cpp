/*
 *	WheelSpeed.cpp - Library for measuring wheel speed.
 *	Created by Rahul Goyal, July 1 2019.
 *	Released to Cal Poly Baja SAE. ;)
 */

#include <Arduino.h>
#include "WheelSpeed.h"

const uint32_t TIMEOUT = 1000000;

// Constructor
WheelSpeed::WheelSpeed(uint8_t triggers) {
	// Initialize variables
	this->triggers = triggers;
	prevTime = micros();
	currTime = micros();
}

void WheelSpeed::calc() {
	prevTime = currTime;
	currTime = micros();
}

float WheelSpeed::get() {
	if (micros() - prevTime >= TIMEOUT) {
		return 0;
	}
	return (float)1000000 / ((currTime - prevTime) * triggers);		// Rotations per Second (RPS)
	// return (float)60000000 / ((currTime - prevTime) * triggers);	// Rotations per Minute (RPM)
}