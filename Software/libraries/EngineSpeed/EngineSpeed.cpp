/*
 *	EngineSpeed.cpp - Library for measuring engine speed.
 *	Created by Rahul Goyal, July 1 2019.
 *	Released to Cal Poly Baja SAE. ;)
 */

#include <Arduino.h>
#include "EngineSpeed.h"

const uint32_t TIMEOUT = 1000000;

// Constructor
EngineSpeed::EngineSpeed(uint8_t triggers) {
	// Initialize variables
	this->triggers = triggers;
	for (uint8_t i = 0; i < 2 * triggers; i++) {
		prevTime[i] = micros();
	}
	currTime = micros();
}

// Interrupt Service Routine Function
/** This function stores the value of currTime (from the previous call of this
	function by the interrupt service routine) in the appropriate position of
	the prevTime array.**/
void EngineSpeed::calc() {
	prevTime[pos] = currTime;
	currTime = micros();
	// Increment position
	pos++;
	if (pos >= 2 * triggers) {
		pos = 0;
	}
}

// Get Engine Speed Function
/** This function uses the currTime and appropriate prevTime to calculate the
	engine speed and return it as a unsigned 16-bit integer. If the time
	difference exceeds the defined TIMEOUT value (in microseconds), a value of
	zero is returned to indicate that the vehicle is not moving.**/
uint16_t EngineSpeed::get() {
	if (micros() - prevTime[pos] >= TIMEOUT) {
		return 0;
	}
	// return 2000000 / (currTime - prevTime[pos]);	// Rotations per Second (RPS)
	return 120000000 / (currTime - prevTime[pos]);	// Rotations per Minute (RPM)
}