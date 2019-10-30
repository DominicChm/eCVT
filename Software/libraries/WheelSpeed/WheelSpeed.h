/*
 *	WheelSpeed.h - Library header for measuring wheel speed.
 *	Created by Rahul Goyal, July 1, 2019.
 *	Released to Cal Poly Baja SAE. ;)
 */

#ifndef WheelSpeed_h
#define WheelSpeed_h

#include <Arduino.h>

class WheelSpeed {

	public:
		// Constructor
		WheelSpeed(uint8_t triggers);

		// Methods
		void calc();
		float get();

	private:
		uint8_t triggers;
		
		volatile uint32_t prevTime;
		volatile uint32_t currTime;
};

#endif