/*
 * eCVT.cpp - Main code to control eCVT.
 * Created by Rahul Goyal, July 2019.
 * Maintained by Rahul Goyal, 2019-21.
 * Maintained by Shaina Bagri, 2020-21.
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
#include "./FSMVars/FSMVars.h"
#include "./Primary/Primary.h"
#include "./Secondary/Secondary.h"
#include "./Engine/Engine.h"
#include "./Communication/Communication.h"
#include "./DashboardLEDs/DashboardLEDs.h"
#include "./HallEffectTask/HallEffectTask.h"
#include "./LaunchControl/LaunchControl.h"



/* ** CONFIG ** */

#include "WiringDec2019.h"
// #include "WiringMar2020.h"


/* ** SYSTEM ** */

FSMVars fsm;

/** ePID will only work with PI or PID control. The integral term is necessary.
	pPID will only work with P-Only or PD control. Do NOT use the integral term.
	sPID will only work with P-Only or PD control. Do NOT use the integral term.
	TODO PID DOCUMENTATION: EFFECT OF CTRL_PERIOD. **/
PIDController ePID(0.5, 0.2, 0);		// Ratio Percent / Revolutions per Minute (%/RPM)
PIDController pPID(0.03,  0, 0);		// Duty Cycle Percent / Encoder Counts (%/Count)
PIDController sPID(0.03,  0, 0);		// Duty Cycle Percent / Encoder Counts (%/Count)

EngineSpeed engineSpeed( 8);
WheelSpeed rWheelsSpeed(24);
// WheelSpeed flWheelSpeed(24);
// WheelSpeed frWheelSpeed(24);

Encoder pEnc(P_ENC_A, P_ENC_B);
Encoder sEnc(S_ENC_A, S_ENC_B);

Motor pMot(P_MOT_INA, P_MOT_INB, P_MOT_PWM);
Motor sMot(S_MOT_INA, S_MOT_INB, S_MOT_PWM);

IntervalTimer ctrlTimer;
IntervalTimer commTimer;
const uint32_t CTRL_PERIOD = 10000;		// Microseconds (us)
const uint32_t COMM_PERIOD = 10000;		// Microseconds (us)



/* ** FINITE STATE MACHINE ** */

Engine engine(fsm, ePID);
Primary primary(fsm, pPID, pEnc, pMot);
Secondary secondary(fsm, sPID, sEnc, sMot);
HallEffectTask hallEffectTask(fsm, engineSpeed, rWheelsSpeed);
LaunchControl launchControl(fsm, LAUNCH_BUTTON, FBRAKE_PRESSURE, RBRAKE_PRESSURE);
DashboardLEDs dashboardLEDs(fsm, UPSHIFT_LED, BKSHIFT_LED);
Communication communication(fsm, engine, primary, secondary);



/* ** TEMP ** */
void  engineSpeedISR();
void rWheelsSpeedISR();
void ctrlISR();
void commISR();



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

	// FSM Variables Setup
	fsm.run   =  true;
	fsm.eCalc = false;
	fsm.pCalc = false;
	fsm.sCalc = false;
	fsm.comm  = false;
	fsm.eSpeed 	   = 0;					// Revolutions per Minute (RPM)
	fsm.rwSpeed    = 0;					// Revolutions per Minute (RPM)
	fsm.pSetpoint  = 0;					// Encoder Counts (~1/3606 of a revolution)
	fsm.sSetpoint  = 0;					// Encoder Counts (~1/3606 of a revolution)
	fsm.ePIDOutput = 0;
	fsm.pPIDOutput = 0;
	fsm.sPIDOutput = 0;

    //  Timers
    commTimer.begin(commISR, COMM_PERIOD);
    ctrlTimer.begin(ctrlISR, CTRL_PERIOD);

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
	engine.run();
	primary.run();
	secondary.run();
	hallEffectTask.run();

	// Bonus Tasks
	launchControl.run();
	// ecvtstatusLED.run();
	dashboardLEDs.run();
	communication.run();
}

/* **INTERRUPT SERVICE ROUTINES** */

// Hall Effect Sensors
void  engineSpeedISR() {  engineSpeed.calc(); }
void rWheelsSpeedISR() { rWheelsSpeed.calc(); }
// void flWheelSpeedISR() { flWheelSpeed.calc(); }
// void frWheelSpeedISR() { frWheelSpeed.calc(); }

// Timers
void ctrlISR() {
	fsm.eCalc = true;
	fsm.pCalc = true;
	fsm.sCalc = true;
}

void commISR() { fsm.comm = true; }