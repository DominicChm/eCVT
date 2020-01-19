/*
 * eCVT.ino - Main code to control eCVT.
 * Created by Rahul Goyal, July 1, 2019.
 * Released to Cal Poly Baja SAE. ;)
 */

// #define DEBUG 1

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

// Primary
const int8_t P_MOT_INA = 18;
const int8_t P_MOT_INB = 19;
const int8_t P_MOT_PWM = 22;
const int8_t P_ENC_A =   24;
const int8_t P_ENC_B =   25;

// Secondary
const int8_t S_MOT_INA = 20;
const int8_t S_MOT_INB = 21;
const int8_t S_MOT_PWM = 23;
const int8_t S_ENC_A =   27;
const int8_t S_ENC_B =   26;



/* ** SYSTEM ** */

const int16_t ENGAGE_SPEED = 45;		// Revolutions per Minute (RPM)
const int16_t SHIFT_SPEED  = 35;		// Revolutions per Minute (RPM)
// TODO DISENGAGEMENT SPEED

const int32_t SHEAVE_OFFSET = 0;

/* TODO PID CONTROLLER GAIN DOCUMENTATION */
// PID Controllers
PIDController ePID(   1, 1, 0);
PIDController pPID(0.01, 0, 0);
PIDController sPID(0.01, 0, 0);

// Hall Effect Sensors
EngineSpeed engineSpeed( 8);
WheelSpeed rWheelsSpeed(24);
WheelSpeed flWheelSpeed(24);
WheelSpeed frWheelSpeed(24);

// Motors
Motor pMot(P_MOT_INA, P_MOT_INB, P_MOT_PWM);
Motor sMot(S_MOT_INA, S_MOT_INB, S_MOT_PWM);

// Encoders
Encoder pEnc(P_ENC_A, P_ENC_B);
Encoder sEnc(S_ENC_A, S_ENC_B);

// Calibration
const uint32_t CALIBRATION_DELAY = 5000;	// Milliseconds (ms)



/* ** FINITE STATE MACHINE ** */

// Timer
IntervalTimer timer;
const uint32_t CONTROLLER_PERIOD = 10000;	// Microseconds (us)

// Inter-Communication Variables
bool run;
bool eCalc, pCalc, sCalc;
int32_t pSetpoint, sSetpoint;

// States
int8_t eState, pState, sState;



/* ** MAIN ** */

void setup() {
	// Serial Monitor
	// #ifdef DEBUG
	Serial.begin(9600);
	while (!Serial) { ; } // Wait for serial port to connect. Needed for native USB.
	Serial.println("Connect the motor wires! Delaying for 2 seconds...");
	delay(2000);
	// #endif

	// Timer Interrupt
	timer.begin(controllerISR, CONTROLLER_PERIOD);

	// Initialize Task States
	eState = 0;
	pState = 0;
	sState = 0;
}

void loop() {
	// Debugging
	#ifdef DEBUG
	Serial.println();
	#endif

	// Tasks
	eCVT();
	primary();
	secondary();

	// Temporary
	Serial.println(engineSpeed.read());
}



/* **TASKS** */

void eCVT() {
	// Debugging
	#ifdef DEBUG
	Serial.print("eState: ");
	Serial.println(eState);
	#endif

	switch (eState) {

		// INITIALIZE
		case 0:
			// Engine Speed Setup
			pinMode(ENGINE_SPEED_PIN, INPUT);
			attachInterrupt(digitalPinToInterrupt(ENGINE_SPEED_PIN), engineSpeedISR, RISING);

			// PID Controller Setup
			ePID.setSetpoint(SHIFT_SPEED);
			ePID.setLoSat(  0);
			ePID.setHiSat(100);
			ePID.reset();

			// Run
			run = true;

			// State Changes
			eState = 1;
			return;

		// DISENGAGED
		case 1:
			pSetpoint = 0;
			sSetpoint = sRatioToTicks(100);

			// State Changes
			// noInterrupts();
			if (engineSpeed.read() > ENGAGE_SPEED && run) {
				// interrupts();
				ePID.reset();
				eState = 2;
			}
			return;

		// ENGAGED, PID CONTROLLER - REST
		case 2:
			// State Changes
			// noInterrupts();
			if (engineSpeed.read() < ENGAGE_SPEED || !run) {
				// interrupts();
				eState = 1;
			} else if (eCalc) {
				eState = 3;
			}
			return;

		// ENGAGED, PID CONTROLLER - UPDATE
		case 3:
			// noInterrupts();
			ePID.calc(engineSpeed.read());
			// interrupts();
			pSetpoint = pRatioToTicks(ePID.get());
			sSetpoint = sRatioToTicks(ePID.get());

			// Debugging
			#ifdef DEBUG
			Serial.print("ePID: ");
			Serial.println(ePID.get());
			#endif

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
	#endif

	static uint32_t pCalTime;			// Milliseconds (ms)

	switch (pState) {
		// INITIALIZE
		case 0:
			// Setup Motor
			pMot.init();
			pMot.setDutyCycle(0);

			// Setup PID Controller
			pPID.setSetpoint(0);
			pPID.setLoSat(-100);
			pPID.setHiSat( 100);
			pPID.reset();

			// State Changes
			pState = 1;
			return;

		// CALIBRATE - OPEN SHEAVES
		case 1:
			pMot.setDutyCycle(-10);
			pCalTime = millis();
			// State Changes
			pState = 2;
			return;

		// CALIBRATE - ZERO ENCODER
		case 2:
			if (millis() - pCalTime > CALIBRATION_DELAY) {
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
			pPID.setSetpoint(pSetpoint + SHEAVE_OFFSET);
			pPID.calc(pEnc.read());
			pMot.setDutyCycle(pPID.get());

			// Debugging
			#ifdef DEBUG
			Serial.print("pPID: ");
			Serial.println(pPID.get());
			#endif

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
	#endif

	static uint32_t sCalTime;			// Milliseconds (ms)
	
	switch (sState) {
		// INITIALIZE
		case 0:
			// Setup Motor
			sMot.init();
			sMot.setDutyCycle(0);

			// Setup PID Controller
			sPID.setSetpoint(0);
			sPID.setLoSat(-100);
			sPID.setHiSat( 100);
			sPID.reset();

			// State Changes
			sState = 1;
			return;

		// CALIBRATE - OPEN SHEAVES
		case 1:
			sMot.setDutyCycle(-10);
			sCalTime = millis();
			// State Changes
			sState = 2;
			return;

		// CALIBRATE - ZERO ENCODER
		case 2:
			if (millis() - sCalTime > CALIBRATION_DELAY) {
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
			sPID.setSetpoint(sSetpoint + SHEAVE_OFFSET);
			sPID.calc(sEnc.read());
			sMot.setDutyCycle(sPID.get());

			// Debugging
			#ifdef DEBUG
			Serial.print("sPID: ");
			Serial.println(sPID.get());
			#endif

			// State Changes
			sCalc = false;
			sState = 3;
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
