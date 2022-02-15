/*
 *	PIDController.h - Library header for PID controller.
 *	Created by Rahul Goyal, July 2019.
 *	Released to Cal Poly Baja SAE. ;)
 */

#ifndef PIDController_h
#define PIDController_h

#include <Arduino.h>

class PIDController
{
public:
	// Constructor
	PIDController(float Kp, float Ki, float Kd);

	// Methods
	void setKp(float Kp);
	void setKi(float Ki);
	void setKd(float Kd);
	void setSetpoint(int32_t setpoint);
	void setLoSat(int16_t loSat);
	void setHiSat(int16_t hiSat);

	void calc(int32_t measurement);
	int16_t get();
	int16_t getP();
	int16_t getI();
	int16_t getD();
	void reset();

private:
	int32_t setpoint = 0;

	int32_t error = 0;
	int32_t integral = 0;
	int32_t derivative = 0;

	int16_t loSat = 0;
	int16_t hiSat = 0;
	bool saturated = false;

	float Kp;
	float Ki;
	float Kd;
};

#endif
