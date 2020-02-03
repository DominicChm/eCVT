/*
 * eCVT.ino - Main code to control eCVT.
 * Created by Rahul Goyal, July 1, 2019.
 * Released to Cal Poly Baja SAE. ;)
 */

#define DEBUG 1

#include <Arduino.h>
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

// Dashboard
const int8_t LAUNCH_CONTROL = 2;
const int8_t UPSHIFT_LED = 3;
const int8_t BKSHIFT_LED = 4;

// Communication
#define DAQ_SERIAL Serial1



/* ** SYSTEM ** */

// PID Controllers
/** ePID will only work with PI or PID control. The integral term is necessary.
	pPID will only work with P-Only or PD control. Do NOT use the integral term.
	sPID will only work with P-Only or PD control. Do NOT use the integral term.
	TODO PID DOCUMENTATION: EFFECT OF CTRL_PERIOD. **/
PIDController ePID(0.5, 0.1, 0);		// Ratio Percent / Revolutions per Minute (%/RPM)
PIDController pPID(0.01, 0, 0);			// Duty Cycle Percent / Encoder Counts (%/Count)
PIDController sPID(0.01, 0, 0);			// Duty Cycle Percent / Encoder Counts (%/Count)

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

// eCVT Shift Curve
/* TODO DISENGAGEMENT SPEED */
const int16_t ENGAGE_SPEED = 2000;		// Revolutions per Minute (RPM)
const int16_t SHIFT_SPEED  = 3400;		// Revolutions per Minute (RPM)

// Primary/Secondary Calibration
const uint32_t CALIB_DELAY = 15000;		// Milliseconds (ms)

// Primary/Secondary Sheave Offset
/** This constant is used to account for mechanical imperfections and adjust belt
	clamping force. Examples of mechanical imperfections include:
		1. belt wear
		2. deflection
		3. manufacturing tolerances
	This constant offsets the ideal sheave position (as determined by the lookup
	table) by a number of encoder counts. A larger number indicates increased
	clamping and a smaller number indicates decreased clamping. The effective
	change in clamping is determined by P * SHEAVE_OFFSET = VOLTS, where P is
	the proportional gain for the respective clutch and VOLTS is the voltage
	applied to the motor at the ideal sheave position. **/
const int32_t SHEAVE_OFFSET = 0;		// Encoder Counts (1/3584 of a revolution)

// Launch Control
const int16_t LC_BRKPRESSURE    = 1640;	// 13-bit ADC (1640/8191 ~= 1/5)
const int16_t LC_ENGINESPEED_LO = 2000;	// Revolutions per Minute (RPM)
const int16_t LC_ENGINESPEED_HI = 3000;	// Revolutions per Minute (RPM)

// Dashboard LEDs
const uint32_t FLASH_PERIOD = 500;		// Milliseconds (ms)

// Timers
IntervalTimer ctrlTimer;
IntervalTimer commTimer;
const uint32_t CTRL_PERIOD = 10000;		// Microseconds (us)
const uint32_t COMM_PERIOD = 10000;		// Microseconds (us)



/* ** FINITE STATE MACHINE ** */

// States
int8_t eState = 0;
int8_t pState = 0;
int8_t sState = 0;
int8_t lState = 0;
int8_t dState = 0;
int8_t cState = 0;

// Inter-Communication Variables
bool run   =  true;
bool eCalc = false;
bool pCalc = false;
bool sCalc = false;
bool comm  = false;
int32_t pSetpoint = 0;					// Encoder Counts (1/3584 of a revolution)
int32_t sSetpoint = 0;					// Encoder Counts (1/3584 of a revolution)



/* ** MAIN ** */

void setup() {

	// Serial Monitor
	// #ifdef DEBUG
	Serial.begin(9600);
	while (!Serial) { }	// Wait for serial port to connect. Needed for native USB only.
	// #endif

	// TEMPORARY
	Serial.println("Connect the motor wires! Delaying for 2 seconds...");
	delay(2000);

	// Hall Effect Sensor Setup
	pinMode( ENGINE_SPEED_PIN, INPUT);
	pinMode(RWHEELS_SPEED_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt( ENGINE_SPEED_PIN),  engineSpeedISR, RISING);
	attachInterrupt(digitalPinToInterrupt(RWHEELS_SPEED_PIN), rWheelsSpeedISR, RISING);

	// Encoder Setup
	// Handled by Encoder constructor! */

	// Motor Setup
	/* Handled by Motor init() function! */

	// Pressure Transducer Setup
	pinMode(FBRAKE_PRESSURE, INPUT);
	pinMode(RBRAKE_PRESSURE, INPUT);

	// Dashboard Setup
	pinMode(LAUNCH_CONTROL, INPUT_PULLUP);
	pinMode(UPSHIFT_LED, OUTPUT);
	pinMode(BKSHIFT_LED, OUTPUT);

	// Timer Interrupt Setup
	ctrlTimer.begin(ctrlISR, CTRL_PERIOD);
	commTimer.begin(commISR, COMM_PERIOD);
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
	// launchcontrol();
	// dashboardLEDs();
	// communication();
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
	int16_t eSpeed = engineSpeed.read();	// Revolutions per Minute (RPM)
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
			sSetpoint = sRatioToCounts(100);

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
			pSetpoint = pRatioToCounts(ePID.get());
			sSetpoint = sRatioToCounts(ePID.get());

			// Reset flag
			eCalc = false;

			// State Changes
			eState = 2;
			return;
	}
}



void primary() {

	static uint32_t pCalTime;			// Milliseconds (ms)

	// Debugging
	#ifdef DEBUG
	Serial.print("pState: ");
	Serial.println(pState);
	Serial.print("pEnc: ");
	Serial.println(pEnc.read());
	Serial.print("pPID: ");
	Serial.println(pPID.get());
	#endif

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
			// State Changes
			if (millis() - pCalTime > CALIB_DELAY) {
				// Finish calibration and change state
				pEnc.write(0);
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

			// Reset flag
			pCalc = false;

			// State Changes
			pState = 3;
			return;
	}
}



void secondary() {

	static uint32_t sCalTime;			// Milliseconds (ms)

	// Debugging
	#ifdef DEBUG
	Serial.print("sState: ");
	Serial.println(sState);
	Serial.print("sEnc: ");
	Serial.println(sEnc.read());
	Serial.print("sPID: ");
	Serial.println(sPID.get());
	#endif
	
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
			// State Changes
			if (millis() - sCalTime > CALIB_DELAY) {
				// Finish calibration and change state
				sEnc.write(0);
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

			// Reset flag
			sCalc = false;

			// State Changes
			sState = 3;
			return;
	}
}



void launchcontrol() {

	// Engine Speed
	noInterrupts();
	int16_t eSpeed = engineSpeed.read();	// Revolutions per Minute (RPM)
	interrupts();

	switch (lState) {

		// INITIALIZE
		case 0:
			// State Changes
			lState = 1;
			return;

		// ECVT ENABLED
		case 1:
			// State Changes
			if (digitalRead(LAUNCH_CONTROL) &&
				analogRead(FBRAKE_PRESSURE) > LC_BRKPRESSURE &&
				analogRead(RBRAKE_PRESSURE) > LC_BRKPRESSURE &&
				eSpeed < LC_ENGINESPEED_LO) {
				// Disable eCVT and change state
				run = false;
				lState = 2;
			}
			return;

		// ECVT DISABLED
		case 2:
			// State Changes
			if (!digitalRead(LAUNCH_CONTROL) &&
				analogRead(FBRAKE_PRESSURE) < LC_BRKPRESSURE &&
				analogRead(RBRAKE_PRESSURE) < LC_BRKPRESSURE &&
				eSpeed > LC_ENGINESPEED_HI) {
				// Enable eCVT and change state
				run = true;
				lState = 1;
			}
			return;
	}
}



void dashboardLEDs() {

	static int16_t prevRatio;			// Ratio Percent (%)
	static int16_t currRatio;			// Ratio Percent (%)
	static uint32_t prevTime;			// Milliseconds (ms)

	// Update previous and current ratios
	prevRatio = currRatio;
	currRatio = pPID.get();

	switch (dState) {

		// INITIALIZE
		case 0:
			// State Changes
			dState = 1;
			return;

		// HUB STATE
		case 1:
			// State Changes
			if (!run) {
				prevTime = millis();
				dState = 2;
			} else if (currRatio > prevRatio) {
				dState = 4;
			} else if (currRatio < prevRatio) {
				dState = 5;
			} else {
				dState = 6;
			}
			return;
		
		// FLASH (BKSHIFT LED ON)
		case 2:
			// Turn on backshift LED, turn off upshift LED
			digitalWrite(BKSHIFT_LED, HIGH);
			digitalWrite(UPSHIFT_LED, LOW);

			// State Changes
			if (run) {
				dState = 1;
			} else if (millis() - prevTime > FLASH_PERIOD) {
				// Increment prevTime by FLASH_PERIOD and change state
				prevTime += FLASH_PERIOD;
				dState = 3;
			}
			return;

		// FLASH (UPSHIFT LED ON)
		case 3:
			// Turn off backshift LED, turn on upshift LED
			digitalWrite(BKSHIFT_LED, LOW);
			digitalWrite(UPSHIFT_LED, HIGH);

			// State Changes
			if (run) {
				dState = 1;
			} else if (millis() - prevTime > FLASH_PERIOD) {
				// Increment prevTime by FLASH_PERIOD and change state
				prevTime += FLASH_PERIOD;
				dState = 2;
			}
			return;

		// BKSHIFT LED ON
		case 4:
			// Turn on backshift LED, turn off upshift LED
			digitalWrite(BKSHIFT_LED, HIGH);
			digitalWrite(UPSHIFT_LED, LOW);

			// State Changes
			dState = 1;
			return;

		// UPSHIFT LED ON
		case 5:
			// Turn off backshift LED, turn on upshift LED
			digitalWrite(BKSHIFT_LED, LOW);
			digitalWrite(UPSHIFT_LED, HIGH);

			// State Changes
			dState = 1;
			return;

		// BOTH LEDS ON
		case 6:
			// Turn on backshift LED, turn on upshift LED
			digitalWrite(BKSHIFT_LED, HIGH);
			digitalWrite(UPSHIFT_LED, HIGH);

			// State Changes
			dState = 1;
			return;
	}
}




void communication() {

	switch (cState) {

		// INITIALIZE
		case 0:
			cState = 1;
			return;

		// IDLE STATE
		case 1:
			return;
	}
}



/* **INTERRUPT SERVICE ROUTINES** */

// Hall Effect Sensors
void  engineSpeedISR() {  engineSpeed.calc(); }
void rWheelsSpeedISR() { rWheelsSpeed.calc(); }
void flWheelSpeedISR() { flWheelSpeed.calc(); }
void frWheelSpeedISR() { frWheelSpeed.calc(); }

// Timers
void ctrlISR() {
	eCalc = true;
	pCalc = true;
	sCalc = true;
}

void commISR() { comm = true; }



/* **LOOKUP TABLES** */

int32_t pRatioToCounts(int16_t ratio) {
	// 1% Ratio Increments
	static const int32_t pLookup[] = {63675,62253,60866,59513,58193,56906,55651,54427,53233,52070,50936,49830,48752,47701,46677,45678,44703,43753,42827,41923,41041,40181,39342,38523,37723,36943,36181,35436,34709,33999,33306,32628,31965,31317,30684,30065,29459,28867,28287,27720,27165,26622,26090,25569,25059,24559,24069,23590,23120,22659,22207,21764,21330,20904,20486,20076,19673,19279,18891,18511,18137,17770,17410,17056,16709,16367,16031,15702,15377,15059,14745,14437,14134,13836,13543,13255,12971,12692,12417,12147,11881,11619,11361,11107,10856,10610,10367,10128,9893,9661,9432,9207,8985,8766,8550,8337,8127,7920,7716,7515,7317};
	if (ratio < 0) { return pLookup[0]; } else if (ratio > 100) { return pLookup[100]; }
	return pLookup[ratio];
}

int32_t sRatioToCounts(int16_t ratio) {
	// 1% Ratio Increments
	static const int32_t sLookup[] = {0,1542,3023,4445,5810,7122,8382,9594,10760,11882,12962,14001,15003,15968,16898,17796,18661,19497,20304,21084,21837,22566,23270,23952,24612,25250,25869,26469,27050,27614,28161,28692,29207,29707,30193,30665,31124,31571,32005,32427,32838,33238,33628,34008,34378,34739,35090,35433,35768,36094,36412,36723,37027,37324,37613,37896,38173,38444,38708,38967,39220,39468,39710,39947,40180,40407,40630,40848,41062,41272,41477,41679,41876,42070,42260,42446,42629,42809,42985,43158,43328,43494,43658,43819,43977,44133,44285,44435,44583,44728,44870,45010,45148,45284,45417,45549,45678,45805,45930,46053,46175};
	if (ratio < 0) { return sLookup[0]; } else if (ratio > 100) { return sLookup[100]; }
	return sLookup[ratio];
}
