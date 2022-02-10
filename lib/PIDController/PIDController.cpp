/*
 *	PIDController.cpp - Library for PID controller.
 *	Created by Rahul Goyal, July 2019.
 *	Released to Cal Poly Baja SAE. ;)
 */

#include <Arduino.h>
#include "PIDController.h"

// Constructor
PIDController::PIDController(float Kp, float Ki, float Kd)
{
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

// Mutator Methods
void PIDController::setKp(float Kp) { this->Kp = Kp; }
void PIDController::setKi(float Ki) { this->Ki = Ki; }
void PIDController::setKd(float Kd) { this->Kd = Kd; }

// Mutator Methods
void PIDController::setSetpoint(int32_t setpoint) { this->setpoint = setpoint; }
void PIDController::setLoSat(int16_t loSat) { this->loSat = loSat; }
void PIDController::setHiSat(int16_t hiSat) { this->hiSat = hiSat; }

// Interrupt Service Routine Method
void PIDController::calc(int32_t measurement)
{
	int32_t prev = error;

	// P
	error = setpoint - measurement;
	// I
	integral += saturated ? 0 : error;
	// D
	derivative = error - prev;
}

// Get output
int16_t PIDController::get()
{
	// Calculate output
	int16_t output = Kp * error + Ki * integral + Kd * derivative;
	// Test if saturated
	saturated = output < loSat || output > hiSat;
	// Return output
	return output;
}

// Get output contribution
int16_t PIDController::getP() { return Kp * error; }
int16_t PIDController::getI() { return Ki * integral; }
int16_t PIDController::getD() { return Kd * derivative; }

// Reset integral and derivative
void PIDController::reset()
{
	integral = 0;
	derivative = 0;
}
