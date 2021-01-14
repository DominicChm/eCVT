/*
 * eCVT.ino - Main code to control eCVT.
 * Created by Rahul Goyal, July 1, 2019.
 * Released to Cal Poly Baja SAE. ;)
 */

// #define DEBUG
#define INFO

#include <Arduino.h>
#include "PIDController.h"
#include "Motor.h"
#include <Encoder.h>
#include "EngineSpeed.h"
#include "WheelSpeed.h"



/* ** CONFIG ** */

#include "WiringDec2019.h"
// #include "WiringMar2020.h"
#include "Communication.h"



/* ** SYSTEM ** */

//FSM Variables
FSMVars fsm;

// PID Controllers
/** ePID will only work with PI or PID control. The integral term is necessary.
	pPID will only work with P-Only or PD control. Do NOT use the integral term.
	sPID will only work with P-Only or PD control. Do NOT use the integral term.
	TODO PID DOCUMENTATION: EFFECT OF CTRL_PERIOD. **/
PIDController ePID(0.5, 0.2, 0);		// Ratio Percent / Revolutions per Minute (%/RPM)
PIDController pPID(0.03,  0, 0);		// Duty Cycle Percent / Encoder Counts (%/Count)
PIDController sPID(0.03,  0, 0);		// Duty Cycle Percent / Encoder Counts (%/Count)

// Hall Effect Sensors
EngineSpeed engineSpeed(fsm, 8);
WheelSpeed rWheelsSpeed(fsm, 24);
// WheelSpeed flWheelSpeed(24);
// WheelSpeed frWheelSpeed(24);

// Encoders
Encoder pEnc(P_ENC_A, P_ENC_B);
Encoder sEnc(S_ENC_A, S_ENC_B);

// Motors
Motor pMot(P_MOT_INA, P_MOT_INB, P_MOT_PWM);
Motor sMot(S_MOT_INA, S_MOT_INB, S_MOT_PWM);

// eCVT Shift Curve
/* TODO DISENGAGEMENT SPEED */
const int16_t ENGAGE_SPEED = 2400;		// Revolutions per Minute (RPM)
const int16_t  SHIFT_SPEED = 3200;		// Revolutions per Minute (RPM)

// eCVT Sheave Offset
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

// Primary/Secondary Calibration
const uint32_t CALIB_DELAY = 10000;		// Milliseconds (ms)
const int16_t CALIB_ESPEED  = 2000;		// Revolutions per Minute (RPM)
const int8_t CALIB_DUTYCYCLE  = 10;		// Magnitude of Duty Cycle Percent (%)

// Primary/Secondary Max Static Duty Cycle
const int8_t MAX_STATIC_DUTYCYCLE = 25;	// Magnitude of Duty Cycle Percent (%)

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

// Inter-Communication Variables
volatile bool run   =  true;
volatile bool eCalc = false;
volatile bool pCalc = false;
volatile bool sCalc = false;
volatile bool comm  = false;
int16_t  eSpeed = 0;					// Revolutions per Minute (RPM)
int16_t rwSpeed = 0;					// Revolutions per Minute (RPM)
int32_t pSetpoint = 0;					// Encoder Counts (~1/3606 of a revolution)
int32_t sSetpoint = 0;					// Encoder Counts (~1/3606 of a revolution)



/* ** TEMP ** */
// (function declarations)
void eCVT();
void primary();
void secondary();
void hallEffectSensors();

void  engineSpeedISR();
void rWheelsSpeedISR();
void ctrlISR();
void commISR();
int32_t pRatioToCounts(int16_t ratio);
int32_t sRatioToCounts(int16_t ratio);



/* ** MAIN ** */

void setup() {

	// Serial Monitor
	#ifdef INFO
	Serial.begin(9600);
	while (!Serial) { }	// Wait for serial port to connect. Needed for native USB only.
	#endif

	// Bench Testing
	#ifdef INFO
	Serial.println("Connect the motor wires!");
	Serial.println("Delaying for 2 seconds..");
	delay(2000);
	Serial.println("GO!");
	#endif

	// Hall Effect Sensor Setup
	pinMode( ENGINE_SPEED_PIN, INPUT);
	pinMode(RWHEELS_SPEED_PIN, INPUT);
	// pinMode(FLWHEEL_SPEED_PIN, INPUT);
	// pinMode(FRWHEEL_SPEED_PIN, INPUT);

	// Encoder Setup
	/* Handled by Encoder constructor! */

	// Motor Setup
	/* Handled by Motor begin() function! */

	// Pressure Transducer Setup
	pinMode(FBRAKE_PRESSURE, INPUT);
	pinMode(RBRAKE_PRESSURE, INPUT);

	// Dashboard Setup
	pinMode(LAUNCH_BUTTON, INPUT_PULLUP);
	pinMode(UPSHIFT_LED, OUTPUT);
	pinMode(BKSHIFT_LED, OUTPUT);

	// **FROM HALL EFFECT TASK **
	attachInterrupt(digitalPinToInterrupt( ENGINE_SPEED_PIN),  engineSpeedISR, RISING);
	attachInterrupt(digitalPinToInterrupt(RWHEELS_SPEED_PIN), rWheelsSpeedISR, RISING);
	// attachInterrupt(digitalPinToInterrupt(FLWHEEL_SPEED_PIN), flWheelSpeedISR, RISING);
	// attachInterrupt(digitalPinToInterrupt(FRWHEEL_SPEED_PIN), frWheelSpeedISR, RISING);
            
}

void loop() {

	// Debugging
	#ifdef DEBUG
	Serial.println();
	#endif

	// Essential Tasks
	eCVT();
	primary();
	secondary();
	hallEffectSensors();

	// Bonus Tasks
	// launchControl();
	// ecvtstatusLED();
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
			pSetpoint = pRatioToCounts(ePID.get()) + SHEAVE_OFFSET;
			sSetpoint = sRatioToCounts(ePID.get()) + SHEAVE_OFFSET;

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
			pMot.setDutyCycle(-CALIB_DUTYCYCLE);
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
				pMot.setDutyCycle(0);
				pState = 3;

				#ifdef INFO
				Serial.println("Finished primary calibration!");
				#endif
			}
			return;

		// CALIBRATE - WAIT FOR USER
		case 3:
			// State Changes
			if (eSpeed > CALIB_ESPEED) {
				pState = 4;
			}
			return;

		// P-ONLY CONTROLLER - REST
		case 4:
			// State Changes
			if (pCalc) {
				pState = 5;
			}
			return;

		// P-ONLY CONTROLLER - UPDATE
		case 5:
			// Update primary setpoint and calculate PID output
			pPID.setSetpoint(pSetpoint);
			pPID.calc(pEnc.read());

			// Set primary duty cycle
			if (eSpeed == 0) {
				pMot.setDutyCycle(min(MAX_STATIC_DUTYCYCLE, pPID.get()));
			} else {
				pMot.setDutyCycle(pPID.get());
			}

			// Reset flag
			pCalc = false;

			// State Changes
			pState = 4;
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
			sMot.setDutyCycle(-CALIB_DUTYCYCLE);
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
				sMot.setDutyCycle(0);
				sState = 3;

				#ifdef INFO
				Serial.println("Finished secondary calibration!");
				#endif
			}
			return;

		// CALIBRATE - WAIT FOR USER
		case 3:
			// State Changes
			if (eSpeed > CALIB_ESPEED) {
				sState = 4;
			}
			return;

		// P-ONLY CONTROLLER - REST
		case 4:
			// State Changes
			if (sCalc) {
				sState = 5;
			}
			return;

		// P-ONLY CONTROLLER - UPDATE
		case 5:
			// Update secondary setpoint and calculate PID output
			sPID.setSetpoint(sSetpoint);
			sPID.calc(sEnc.read());

			// Set secondary duty cycle
			if (rwSpeed == 0) {
				sMot.setDutyCycle(min(MAX_STATIC_DUTYCYCLE, sPID.get()));
			} else {
				sMot.setDutyCycle(sPID.get());
			}

			// Reset flag
			sCalc = false;

			// State Changes
			sState = 4;
			return;
	}
}

void hallEffectSensors() {

	static int8_t state = 0;

	switch (state) {

		// INITIALIZE
		case 0:
			// Attach interrupts
			attachInterrupt(digitalPinToInterrupt( ENGINE_SPEED_PIN),  engineSpeedISR, RISING);
			attachInterrupt(digitalPinToInterrupt(RWHEELS_SPEED_PIN), rWheelsSpeedISR, RISING);
			// attachInterrupt(digitalPinToInterrupt(FLWHEEL_SPEED_PIN), flWheelSpeedISR, RISING);
			// attachInterrupt(digitalPinToInterrupt(FRWHEEL_SPEED_PIN), frWheelSpeedISR, RISING);

			// State Changes
			state = 1;
			return;

		// UPDATE
		case 1:
			// Read engine speed
			cli();
			eSpeed  =  engineSpeed.read();
			sei();

			// Read rear wheel speed
			cli();
			rwSpeed = rWheelsSpeed.read();
			sei();

			// // Read front left wheel speed
			// cli();
			// flSpeed = flWheelSpeed.read();
			// sei();

			// // Read front right wheel speed
			// cli();
			// frSpeed = frWheelSpeed.read();
			// sei();

			// State Changes
			return;
	}

}

void launchControl() {

	static int8_t state = 0;

	switch (state) {

		// INITIALIZE
		case 0:
			// State Changes
			state = 1;
			return;

		// ECVT ENABLED
		case 1:
			// State Changes
			/** A low signal from the launch button means the button is pressed
				because the pin mode is configured to input pullup. **/
			if (!digitalRead(LAUNCH_BUTTON) &&
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
			/** A high signal from the launch button means the button is not
				pressed because the pin mode is configured to input pullup. **/
			if (digitalRead(LAUNCH_BUTTON) &&
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
	currRatio = ePID.get();

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

void communication() {

	static int8_t state = 0;

	// Structure of Data
	static struct Data {
		uint32_t time;
		// eCVT
		int8_t  eState;
		int16_t eSpeed;
		int16_t ePID;
		int16_t eP;
		int16_t eI;
		int16_t eD;
		// Primary
		int8_t  pState;
		int32_t pSet;
		int32_t pEnc;
		int16_t pPID;
		// Secondary
		int8_t  sState;
		int32_t sSet;
		int32_t sEnc;
		int16_t sPID;
	} data;

	static int8_t numBytesWritten = 0;

	switch (state) {

		// INITIALIZE
		case 0:
			// Timer Interrupt Setup
			commTimer.begin(commISR, COMM_PERIOD);

			// State Changes
			state = 1;
			return;

		// WRITE START DATA
		case 1:
			// Write start data
			if (comm) {
				Serial.write(START_BYTE_VAL);
				numBytesWritten++;
			}

			// State Changes
			if (numBytesWritten >= 2) {
				state = 2;
			}
			return;

		// STORE ECVT DATA
		case 2:
			// Store time data
			data.time = micros();

			// Store eCVT task data
			data.eState = eState;
			data.eSpeed = eSpeed;
			data.ePID = ePID.get();
			data.eP = ePID.getP();
			data.eI = ePID.getI();
			data.eD = ePID.getD();

			// Store primary task data
			data.pState = pState;
			data.pSet = pSetpoint;
			data.pEnc = pEnc.read();
			data.pPID = pPID.get();

			// Store secondary task data
			data.pState = pState;
			data.pSet = pSetpoint;
			data.pEnc = pEnc.read();
			data.pPID = pPID.get();

			// State Changes
			state = 3;
			return;
		
		// WRITE ECVT DATA
		case 3:
			return;
	}
}



/* **INTERRUPT SERVICE ROUTINES** */

// Hall Effect Sensors
void  engineSpeedISR() {  engineSpeed.calc(); }
void rWheelsSpeedISR() { rWheelsSpeed.calc(); }
// void flWheelSpeedISR() { flWheelSpeed.calc(); }
// void frWheelSpeedISR() { frWheelSpeed.calc(); }

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
