/*
 * eCVT.ino - Main code to control eCVT.
 * Created by Rahul Goyal, July 1, 2019.
 * Released to Cal Poly Baja SAE. ;)
 */

#define DEBUG 1

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "PIDController.h"
#include "Motor.h"
#include <Encoder.h>
#include "EngineSpeed.h"
#include "WheelSpeed.h"



/* ** WIRING ** */

// Hall Effect Sensors
const int8_t  ENGINE_SPEED_PIN =  5;
const int8_t RWHEELS_SPEED_PIN =  6;
const int8_t FLWHEEL_SPEED_PIN = 29;
const int8_t FRWHEEL_SPEED_PIN = 30;

// Encoders
/* Swap A and B pins to swap direction. */
const int8_t P_ENC_A = 24;
const int8_t P_ENC_B = 25;
const int8_t S_ENC_A = 27;
const int8_t S_ENC_B = 26;

// Motors
/* Swap A and B pins to swap direction. */
const int8_t P_MOT_INA = 18;
const int8_t P_MOT_INB = 19;
const int8_t P_MOT_PWM = 22;
const int8_t S_MOT_INA = 20;
const int8_t S_MOT_INB = 21;
const int8_t S_MOT_PWM = 23;

// Pressure Transducers
const int8_t FBRAKE_PRESSURE = 34;
const int8_t RBRAKE_PRESSURE = 33;

// Launch Control Button
const int8_t LAUNCH_CONTROL = 2;

// Upshift/Backshift LEDs
const int8_t UPSHIFT_LED = 3;
const int8_t BKSHIFT_LED = 4;



/* ** SYSTEM ** */

const int16_t ENGAGE_SPEED = 2000;		// Revolutions per Minute (RPM)
const int16_t SHIFT_SPEED  = 3400;		// Revolutions per Minute (RPM)
// TODO DISENGAGEMENT SPEED

/** This constant is used to account for mechanical imperfections and adjust belt
	clamping force. Examples of mechanical imperfections include:
		1. belt wear
		2. deflection
		3. manufacturing tolerances
	This constant offsets the ideal sheave position (as determined by the lookup
	table) by a number of encoder ticks. A larger number indicates increased
	clamping and a smaller number indicates decreased clamping. The effective
	change in clamping is determined by P * SHEAVE_OFFSET = VOLTS, where P is
	the proportional gain for the respective clutch and VOLTS is the voltage
	applied to the motor at the ideal sheave position. **/
const int32_t SHEAVE_OFFSET = 0;		// Encoder Ticks (1/3584 of a revolution)

/* ePID will only work with PI or PID control. The integral term is necessary. */
/* pPID will only work with P-Only or PD control. Do NOT use the integral term. */
/* sPID will only work with P-Only or PD control. Do NOT use the integral term. */
// PID Controllers
PIDController ePID(   1, 1, 0);
PIDController pPID(0.01, 0, 0);
PIDController sPID(0.01, 0, 0);

// Hall Effect Sensors
EngineSpeed engineSpeed( 8);
WheelSpeed rWheelsSpeed(24);
WheelSpeed flWheelSpeed(24);
WheelSpeed frWheelSpeed(24);

// Encoders
Encoder pEnc(P_ENC_A, P_ENC_B);
Encoder sEnc(S_ENC_A, S_ENC_B);

// Motors
Motor pMot(P_MOT_INA, P_MOT_INB, P_MOT_PWM);
Motor sMot(S_MOT_INA, S_MOT_INB, S_MOT_PWM);

// Calibration
const uint32_t CALIBRATION_DELAY = 15000;	// Milliseconds (ms)



/* ** FINITE STATE MACHINE ** */

// States
int8_t eState = 0;
int8_t pState = 0;
int8_t sState = 0;
int8_t aState = 0;
int8_t bState = 0;
int8_t cState = 0;

// Inter-Communication Variables
bool run   = true;
bool eCalc = false;
bool pCalc = false;
bool sCalc = false;
int32_t pSetpoint = 0;
int32_t sSetpoint = 0;

// Timer
IntervalTimer timer;
const uint32_t CONTROLLER_PERIOD = 10000;	// Microseconds (us)



/* ** MAIN ** */

void setup() {
	// Serial Monitor
	// #ifdef DEBUG
	Serial.begin(9600);
	while (!Serial) { }	// Wait for serial port to connect. Needed for native USB.
	// #endif

	// TEMPORARY
	Serial.println("Connect the motor wires! Delaying for 2 seconds...");
	delay(2000);

	// Hall Effect Sensor Setup
	pinMode(ENGINE_SPEED_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(ENGINE_SPEED_PIN), engineSpeedISR, RISING);
	pinMode(RWHEELS_SPEED_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(RWHEELS_SPEED_PIN), rWheelsSpeedISR, RISING);

	// Pressure Transducer Setup
	pinMode(FBRAKE_PRESSURE, INPUT);
	pinMode(RBRAKE_PRESSURE, INPUT);

	// Launch Control Button Setup
	pinMode(LAUNCH_CONTROL, INPUT);

	// Upshift/Backshift LEDs Setup
	pinMode(UPSHIFT_LED, OUTPUT);
	pinMode(BKSHIFT_LED, OUTPUT);

	// Timer Interrupt Setup
	timer.begin(controllerISR, CONTROLLER_PERIOD);
}

void loop() {
	// Debugging
	#ifdef DEBUG
	Serial.println();
	#endif

	// TEMPORARY
	Serial.println(engineSpeed.read());

	// Tasks
	eCVT();
	primary();
	secondary();
}



/* **TASKS** */

void eCVT() {
	// Debugging
	#ifdef DEBUG
	Serial.print("eState: ");
	Serial.println(eState);
	Serial.print("ePID: ");
	Serial.println(ePID.get());
	#endif

	// Engine Speed
	noInterrupts();
	int16_t eSpeed = engineSpeed.read();
	interrupts();

	switch (eState) {

		// INITIALIZE
		case 0:
			// PID Controller Setup
			ePID.setSetpoint(SHIFT_SPEED);
			ePID.setLoSat(  0);
			ePID.setHiSat(100);
			ePID.reset();

			// State Changes
			eState = 1;
			return;

		// DISENGAGED
		case 1:
			// Set primary and secondary setpoints
			pSetpoint = 0;
			sSetpoint = sRatioToTicks(100);

			// State Changes
			if (eSpeed > ENGAGE_SPEED && run) {
				ePID.reset();
				eState = 2;
			}
			return;

		// ENGAGED, PID CONTROLLER - REST
		case 2:
			// State Changes
			if (eSpeed < ENGAGE_SPEED || !run) {
				eState = 1;
			} else if (eCalc) {
				eState = 3;
			}
			return;

		// ENGAGED, PID CONTROLLER - UPDATE
		case 3:
			// Calculate PID output
			ePID.calc(eSpeed);

			// Set primary and secondary setpoints
			pSetpoint = pRatioToTicks(ePID.get());
			sSetpoint = sRatioToTicks(ePID.get());

			// State Changes
			eCalc = false;
			eState = 2;
			return;
	}
}



void primary() {
	// Debugging
	#ifdef DEBUG
	Serial.print("pState: ");
	Serial.println(pState);
	Serial.print("pEnc: ");
	Serial.println(pEnc.read());
	Serial.print("pPID: ");
	Serial.println(pPID.get());
	#endif

	static uint32_t pCalTime;			// Milliseconds (ms)

	switch (pState) {
		// INITIALIZE
		case 0:
			// Motor Setup
			pMot.init();
			pMot.setDutyCycle(0);

			// PID Controller Setup
			pPID.setSetpoint(0);
			pPID.setLoSat(-100);
			pPID.setHiSat( 100);
			pPID.reset();

			// State Changes
			pState = 1;
			return;

		// CALIBRATE - OPEN SHEAVES
		case 1:
			// Start calibration
			pMot.setDutyCycle(-10);
			pCalTime = millis();

			// State Changes
			pState = 2;
			return;

		// CALIBRATE - ZERO ENCODER
		case 2:
			if (millis() - pCalTime > CALIBRATION_DELAY) {
				// Finish calibration
				pEnc.write(0);
				
				// State Changes
				pState = 3;
			}
			return;

		// P-ONLY CONTROLLER - REST
		case 3:
			// State Changes
			if (pCalc) {
				pState = 4;
			}
			return;

		// P-ONLY CONTROLLER - UPDATE
		case 4:
			// Update primary setpoint and calculate PID output
			pPID.setSetpoint(pSetpoint + SHEAVE_OFFSET);
			pPID.calc(pEnc.read());

			// Set primary duty cycle
			pMot.setDutyCycle(pPID.get());

			// State Changes
			pCalc = false;
			pState = 3;
			return;
	}
}



void secondary() {
	// Debugging
	#ifdef DEBUG
	Serial.print("sState: ");
	Serial.println(sState);
	Serial.print("sEnc: ");
	Serial.println(sEnc.read());
	Serial.print("sPID: ");
	Serial.println(sPID.get());
	#endif

	static uint32_t sCalTime;			// Milliseconds (ms)
	
	switch (sState) {
		// INITIALIZE
		case 0:
			// Motor Setup
			sMot.init();
			sMot.setDutyCycle(0);

			// PID Controller Setup
			sPID.setSetpoint(0);
			sPID.setLoSat(-100);
			sPID.setHiSat( 100);
			sPID.reset();

			// State Changes
			sState = 1;
			return;

		// CALIBRATE - OPEN SHEAVES
		case 1:
			// Start calibration
			sMot.setDutyCycle(-10);
			sCalTime = millis();

			// State Changes
			sState = 2;
			return;

		// CALIBRATE - ZERO ENCODER
		case 2:
			if (millis() - sCalTime > CALIBRATION_DELAY) {
				// Finish calibration
				sEnc.write(0);

				// State Changes
				sState = 3;
			}
			return;

		// P-ONLY CONTROLLER - REST
		case 3:
			// State Changes
			if (sCalc) {
				sState = 4;
			}
			return;

		// P-ONLY CONTROLLER - UPDATE
		case 4:
			// Update secondary setpoint and calculate PID output
			sPID.setSetpoint(sSetpoint + SHEAVE_OFFSET);
			sPID.calc(sEnc.read());

			// Set secondary duty cycle
			sMot.setDutyCycle(sPID.get());

			// State Changes
			sCalc = false;
			sState = 3;
			return;
	}
}



void launchControl() {
	switch (aState) {
		// INITIALIZE
		case 0:
			// State Changes
			aState = 1;
			return;
		// HUB STATE
		case 1:
			return;
		}
}



void statusLEDs() {
	switch (bState) {
		// INITIALIZE
		case 0:
			// State Changes
			bState = 1;
			return;
		// HUB STATE
		case 1:
			return;
		}
}




void communication() {
	switch (cState) {
		// INITIALIZE
		case 0:
			cState = 1;
			return;
		// HUB STATE
		case 1:
			return;
		}
}



/* **INTERRUPT SERVICE ROUTINES** */

void  engineSpeedISR() {  engineSpeed.calc(); }
void rWheelsSpeedISR() { rWheelsSpeed.calc(); }
void flWheelSpeedISR() { flWheelSpeed.calc(); }
void frWheelSpeedISR() { frWheelSpeed.calc(); }
void   controllerISR() {
	eCalc = true;
	pCalc = true;
	sCalc = true;
}



/* **LOOKUP TABLES** */

int32_t pRatioToTicks(float ratio) {
	// 1% ratio increments
	static const int32_t pLookup[] = {63675,62253,60866,59513,58193,56906,55651,54427,53233,52070,50936,49830,48752,47701,46677,45678,44703,43753,42827,41923,41041,40181,39342,38523,37723,36943,36181,35436,34709,33999,33306,32628,31965,31317,30684,30065,29459,28867,28287,27720,27165,26622,26090,25569,25059,24559,24069,23590,23120,22659,22207,21764,21330,20904,20486,20076,19673,19279,18891,18511,18137,17770,17410,17056,16709,16367,16031,15702,15377,15059,14745,14437,14134,13836,13543,13255,12971,12692,12417,12147,11881,11619,11361,11107,10856,10610,10367,10128,9893,9661,9432,9207,8985,8766,8550,8337,8127,7920,7716,7515,7317};
	if (ratio < 0) { return pLookup[0]; } else if (ratio > 100) { return pLookup[100]; }
	return pLookup[(int8_t)ratio];
}

int32_t sRatioToTicks(float ratio) {
	// 1% ratio increments
	static const int32_t sLookup[] = {0,1542,3023,4445,5810,7122,8382,9594,10760,11882,12962,14001,15003,15968,16898,17796,18661,19497,20304,21084,21837,22566,23270,23952,24612,25250,25869,26469,27050,27614,28161,28692,29207,29707,30193,30665,31124,31571,32005,32427,32838,33238,33628,34008,34378,34739,35090,35433,35768,36094,36412,36723,37027,37324,37613,37896,38173,38444,38708,38967,39220,39468,39710,39947,40180,40407,40630,40848,41062,41272,41477,41679,41876,42070,42260,42446,42629,42809,42985,43158,43328,43494,43658,43819,43977,44133,44285,44435,44583,44728,44870,45010,45148,45284,45417,45549,45678,45805,45930,46053,46175};
	if (ratio < 0) { return sLookup[0]; } else if (ratio > 100) { return sLookup[100]; }
	return sLookup[(int8_t)ratio];
}
