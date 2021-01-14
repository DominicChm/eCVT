/*
 *	EngineSpeed.h - Library header for measuring engine speed.
 *	Created by Rahul Goyal, July 2019.
 *  Released to Cal Poly Baja SAE. ;)
 */

#ifndef EngineSpeed_h
#define EngineSpeed_h

#include <Arduino.h>
#include "Sensor.h"

class EngineSpeed: public Sensor {

	public:
		// Constructor
		EngineSpeed(FSMVars fsm, int8_t triggers);

		// Methods
		void calc();
		int16_t read();

	private:
		int8_t triggers;
		volatile int8_t pos = 0;

		volatile uint32_t prevTime[100];
		volatile uint32_t currTime;
};

#endif