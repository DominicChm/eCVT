/*
 * eCVT.ino - Main code to control eCVT.
 * Created by Rahul Goyal, July 1, 2019.
 * Released to Cal Poly Baja SAE. ;)
 */

// #define DEBUG 0

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
const int8_t P_MOT_INA = 19;
const int8_t P_MOT_INB = 18;
const int8_t P_MOT_PWM = 22;
const int8_t S_MOT_INA = 20;
const int8_t S_MOT_INB = 21;
const int8_t S_MOT_PWM = 23;

// Pressure Transducers
const int8_t FBRAKE_PRESSURE = 34;
const int8_t RBRAKE_PRESSURE = 33;

// Dashboard
const int8_t LOGGER_BUTTON = 2;
const int8_t MARKER_BUTTON = 3;
const int8_t STATUS_LED = 4;
const int8_t LAUNCH_CONTROL = 2;
const int8_t UPSHIFT_LED = 3;
const int8_t BKSHIFT_LED = 4;

// Communication
#define DAQ_ECVT_SERIAL Serial1
#define DAQ_XBEE_SERIAL Serial2



/* ** SYSTEM ** */

// PID Controllers
/** ePID will only work with PI or PID control. The integral term is necessary.
	pPID will only work with P-Only or PD control. Do NOT use the integral term.
	sPID will only work with P-Only or PD control. Do NOT use the integral term.
	TODO PID DOCUMENTATION: EFFECT OF CTRL_PERIOD. **/
PIDController ePID(0.5, 0.2, 0);		// Ratio Percent / Revolutions per Minute (%/RPM)
PIDController pPID(0.03, 0, 0);			// Duty Cycle Percent / Encoder Counts (%/Count)
PIDController sPID(0.03, 0, 0);			// Duty Cycle Percent / Encoder Counts (%/Count)

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
const int16_t SHIFT_SPEED  = 3200;		// Revolutions per Minute (RPM)

// Primary/Secondary Calibration
const uint32_t CALIB_DELAY = 10000;		// Milliseconds (ms)

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
const int32_t SHEAVE_OFFSET = 1000;		// Encoder Counts (1/3606 of a revolution)

// Launch Control
const int16_t LC_BRKPRESSURE    = 1640;	// 13-bit ADC (1640/8191 ~= 1/5)
const int16_t LC_ENGINESPEED_LO = 2000;	// Revolutions per Minute (RPM)
const int16_t LC_ENGINESPEED_HI = 3000;	// Revolutions per Minute (RPM)

// Dashboard LEDs
const uint32_t FLASH_PERIOD = 500;		// Milliseconds (ms)

// Communication
const int8_t ECVT_DATA_SIZE = 37;		// Bytes
const int8_t START_DATA_SIZE = 2;		// Bytes
const int8_t CHECK_DATA_SIZE = 2;		// Bytes
const int16_t START_BYTE_VAL = 0x5555;	// 0101 0101

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

// Inter-Communication Variables
volatile bool run   =  true;
volatile bool eCalc = false;
volatile bool pCalc = false;
volatile bool sCalc = false;
volatile bool comm  = false;
int32_t pSetpoint = 0;					// Encoder Counts (1/3606 of a revolution)
int32_t sSetpoint = 0;					// Encoder Counts (1/3606 of a revolution)



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
	Serial.println("GO!");

	// Hall Effect Sensor Setup
	pinMode( ENGINE_SPEED_PIN, INPUT);
	pinMode(RWHEELS_SPEED_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt( ENGINE_SPEED_PIN),  engineSpeedISR, RISING);
	attachInterrupt(digitalPinToInterrupt(RWHEELS_SPEED_PIN), rWheelsSpeedISR, RISING);

	// Encoder Setup
	// Handled by Encoder constructor! */

	// Motor Setup
	/* Handled by Motor begin() function! */

	// Pressure Transducer Setup
	pinMode(FBRAKE_PRESSURE, INPUT);
	pinMode(RBRAKE_PRESSURE, INPUT);

	// Dashboard Setup
	pinMode(LAUNCH_CONTROL, INPUT_PULLUP);
	pinMode(UPSHIFT_LED, OUTPUT);
	pinMode(BKSHIFT_LED, OUTPUT);
}

void loop() {

	// Debugging
	#ifdef DEBUG
	Serial.println();
	#endif

	// TEMPORARY
	// Serial.println(engineSpeed.read());

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

			// Timer Interrupt Setup
			ctrlTimer.begin(ctrlISR, CTRL_PERIOD);

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
			pMot.begin();
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
			sMot.begin();
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

	static int8_t state = 0;

	// Engine Speed
	noInterrupts();
	int16_t eSpeed = engineSpeed.read();	// Revolutions per Minute (RPM)
	interrupts();

	switch (state) {

		// INITIALIZE
		case 0:
			// State Changes
			state = 1;
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
				state = 2;
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
				state = 1;
			}
			return;
	}
}



void dashboardLEDs() {

	static int8_t state = 0;

	static int16_t prevRatio;			// Ratio Percent (%)
	static int16_t currRatio;			// Ratio Percent (%)
	static uint32_t prevTime;			// Milliseconds (ms)

	// Update previous and current ratios
	prevRatio = currRatio;
	currRatio = pPID.get();

	switch (state) {

		// INITIALIZE
		case 0:
			// State Changes
			state = 1;
			return;

		// HUB STATE
		case 1:
			// State Changes
			if (!run) {
				prevTime = millis();
				state = 5;
			} else if (currRatio > prevRatio) {
				state = 2;
			} else if (currRatio < prevRatio) {
				state = 3;
			} else {
				state = 4;
			}
			return;

		// BKSHIFT LED ON
		case 2:
			// Turn on backshift LED, turn off upshift LED
			digitalWrite(BKSHIFT_LED, HIGH);
			digitalWrite(UPSHIFT_LED, LOW);

			// State Changes
			state = 1;
			return;

		// UPSHIFT LED ON
		case 3:
			// Turn off backshift LED, turn on upshift LED
			digitalWrite(BKSHIFT_LED, LOW);
			digitalWrite(UPSHIFT_LED, HIGH);

			// State Changes
			state = 1;
			return;

		// BOTH LEDS ON
		case 4:
			// Turn on backshift LED, turn on upshift LED
			digitalWrite(BKSHIFT_LED, HIGH);
			digitalWrite(UPSHIFT_LED, HIGH);

			// State Changes
			state = 1;
			return;

		// FLASH (BKSHIFT LED ON)
		case 5:
			// Turn on backshift LED, turn off upshift LED
			digitalWrite(BKSHIFT_LED, HIGH);
			digitalWrite(UPSHIFT_LED, LOW);

			// State Changes
			if (run) {
				state = 1;
			} else if (millis() - prevTime > FLASH_PERIOD) {
				// Increment prevTime by FLASH_PERIOD and change state
				prevTime += FLASH_PERIOD;
				state = 6;
			}
			return;

		// FLASH (UPSHIFT LED ON)
		case 6:
			// Turn off backshift LED, turn on upshift LED
			digitalWrite(BKSHIFT_LED, LOW);
			digitalWrite(UPSHIFT_LED, HIGH);

			// State Changes
			if (run) {
				state = 1;
			} else if (millis() - prevTime > FLASH_PERIOD) {
				// Increment prevTime by FLASH_PERIOD and change state
				prevTime += FLASH_PERIOD;
				state = 5;
			}
			return;
	}
}



// void communication() {

// 	static int8_t state = 0;

// 	// Structure of Data
// 	static struct data {
// 	    uint32_t relTime;
// 	    int16_t  eSpeed;
// 	    int16_t ewSpeed;
// 	    int16_t rwSpeed;
// 	    int16_t flSpeed;
// 	    int16_t frSpeed;
// 	    int16_t fPressure;
// 	    int16_t rPressure;
// 	    bool markerButton;
// 	    uint8_t eCVT[ECVT_DATA_SIZE];
// 	};

// 	static int8_t numBytesWritten = 0;

// 	switch (state) {

// 		// INITIALIZE
// 		case 0:
// 			// Timer Interrupt Setup
// 			commTimer.begin(commISR, COMM_PERIOD);

// 			// State Changes
// 			state = 1;
// 			return;

// 		// WRITE START DATA
// 		case 1:
// 			// Write start data
// 			if (comm) {
// 				Serial.write(0x5555);
// 				numBytesWritten++;
// 				comm = false;
// 			}

// 			// State Changes
// 			if (numBytesWritten >= 2) {
// 				state = 2;
// 			}
// 			return;

// 		// STORE ECVT DATA
// 		case 2:
// 			return;
		
// 		// WRITE ECVT DATA
// 		case 3:
// 			return;
// 	}
// }



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
	static const int32_t pLookup[] = {64546,63144,61773,60433,59124,57845,56597,55378,54187,53026,51892,50786,49706,48653,47625,46622,45644,44689,43757,42847,41959,41093,40247,39421,38614,37827,37057,36306,35572,34854,34153,33468,32798,32143,31503,30877,30264,29665,29078,28504,27942,27393,26854,26327,25810,25304,24809,24323,23847,23380,22923,22474,22035,21603,21180,20765,20358,19958,19565,19180,18802,18430,18066,17707,17355,17010,16670,16336,16008,15685,15368,15056,14749,14448,14151,13859,13572,13290,13012,12738,12469,12203,11942,11685,11432,11183,10937,10696,10457,10223,9991,9763,9539,9317,9099,8884,8672,8463,8256,8053,7852};
	if (ratio < 0) { return pLookup[0]; } else if (ratio > 100) { return pLookup[100]; }
	return pLookup[ratio];
}

int32_t sRatioToCounts(int16_t ratio) {
	// 1% Ratio Increments
	static const int32_t sLookup[] = {0,1549,3036,4463,5834,7150,8415,9631,10800,11924,13006,14047,15050,16015,16945,17842,18707,19541,20346,21123,21874,22599,23301,23978,24634,25269,25883,26478,27054,27613,28154,28679,29188,29683,30163,30629,31082,31522,31949,32365,32770,33164,33547,33920,34283,34637,34982,35318,35646,35965,36277,36581,36878,37168,37451,37727,37997,38261,38519,38771,39018,39259,39495,39725,39951,40172,40389,40601,40808,41012,41211,41406,41598,41785,41969,42150,42327,42500,42671,42838,43002,43163,43321,43477,43629,43779,43926,44071,44213,44353,44490,44625,44758,44888,45017,45143,45267,45389,45509,45628,45744};
	if (ratio < 0) { return sLookup[0]; } else if (ratio > 100) { return sLookup[100]; }
	return sLookup[ratio];
}
