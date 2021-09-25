/*
 *	EngineSpeed.cpp - Library for measuring engine speed.
 *	Created by Rahul Goyal, July 2019.
 *	Released to Cal Poly Baja SAE. ;)
 */

#include <Arduino.h>
#include "EngineSpeed.h"

const uint32_t TIMEOUT = 1000000;

// Constructor
/** This constructor accepts the number of triggers per revolution and stores
	it. The appropriate number of positions in the prevTime array and currTime
	are initialized to the system time. **/
EngineSpeed::EngineSpeed(int8_t triggers)
{
	// Initialize variables
	this->triggers = triggers;
	for (int8_t i = 0; i < 2 * triggers; i++)
	{
		prevTime[i] = micros();
	}
	currTime = micros();
}

// Interrupt Service Routine Method
/** This function stores the value of currTime (from the previous call to this
	function by the interrupt service routine) in the appropriate position of
	the prevTime array and currTime is updated to the system time. Afterwards,
	the position is incremented to prepare for the next call to this function.
	If the position exceeds the twice the number of triggers per revolution
	(that is, one thermodynamic cycle of a four-stroke engine), then the
	position is reset to zero. This information is used by the read method to
	calculate the wheel speed. **/
void EngineSpeed::calc()
{
	prevTime[pos] = currTime;
	currTime = micros();
	// Increment position
	pos++;
	if (pos >= 2 * triggers)
	{
		pos = 0;
	}
}

// Read Engine Speed Method
/** This function uses the value of currTime and the value from the appropriate
	position of prevTime to calculate the engine speed and return it as an
	16-bit integer. If the time difference exceeds the defined TIMEOUT constant
	(in microseconds), a value of zero is returned to indicate that engine is
	turned off. **/
int16_t EngineSpeed::read()
{
	if (micros() - prevTime[pos] >= TIMEOUT)
	{
		return 0;
	}
	// return 2000000 / (currTime - prevTime[pos]);	// Revolutions per Second (RPS)
	return 120000000 / (currTime - prevTime[pos]); // Revolutions per Minute (RPM)
}
