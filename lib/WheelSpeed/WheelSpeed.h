/*
 *	WheelSpeed.h - Library header for measuring wheel speed.
 *	Created by Rahul Goyal, July 2019.
 *	Released to Cal Poly Baja SAE. ;)
 */

#ifndef WheelSpeed_h
#define WheelSpeed_h

#include "Sensor.h"
#include <Arduino.h>

class WheelSpeed: public Sensor {

	public:
		// Constructor
		WheelSpeed(int8_t triggers);

		// Methods
		void calc();
		int16_t read();

	private:
		int8_t triggers;
		
		volatile uint32_t prevTime;
		volatile uint32_t currTime;
};

#endif