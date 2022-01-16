/*
 * main.cpp - Main code to control eCVT.
 * Created by Rahul Goyal, July 2019.
 * Maintained by Rahul Goyal, 2019-21.
 * Maintained by Shaina Bagri, 2020-21.
 * Released to Cal Poly Baja SAE. ;)
 */

// #define DEBUG
#define INFO

// Libraries
#include <Arduino.h>
#include "PIDController.h"
#include "Motor.h"
#include <Encoder.h>
#include "LoadCell.h"
#include "EngineSpeed.h"
#include "WheelSpeed.h"

// FSM Tasks
#include "./FSMVars/FSMVars.h"
#include "./Engine/Engine.h"
#include "./Primary/Primary.h"
#include "./Secondary/Secondary.h"
#include "./HallEffectTask/HallEffectTask.h"
#include "./PressureTransducerTask/PressureTransducerTask.h"
#include "./LaunchControl/LaunchControl.h"
#include "./DashboardLEDs/DashboardLEDs.h"
#include "./Communication/Communication.h"

/* ** WIRING ** */

/** _RULE: Do NOT include elsewhere. Pass I/O values/objects as parameters. **/
/*
#include "WiringDec2019.h"
#include "WiringMar2020.h"
*/
#include "WiringDec2021.h"

/* ** SYSTEM ** */

/** ePID will only work with PI or PID control. The integral term is necessary.
    pEncPID will only work with P-Only or PD control. Do NOT use the integral term.
    sEncPID will only work with P-Only or PD control. Do NOT use the integral term.
    sLcPID will work with P-Only, PI, PD, or PID control. A weak integral term is recommended.

    P = Kp * error
    I = Ki * error * dt
    D = Kd * error / dt

    For performance, the PIDController class assumes a constant dt.
    (dt is integral/derivative delta time, defined by CTRL_PERIOD.)
    Therefore:
    - The   integral gain (Ki) must decrease linearly with CTRL_PERIOD.
    - The derivative gain (Kd) must increase linearly with CTRL_PERIOD.
**/
PIDController ePID(0.5, 0.2, 0);   // Ratio Percent / Revolutions per Minute (%/RPM)
PIDController pEncPID(0.03, 0, 0); // Duty Cycle Percent / Encoder Counts (%/Count)
PIDController sEncPID(0.03, 0, 0); // Duty Cycle Percent / Encoder Counts (%/Count)
PIDController sLcPID(0.2, 0, 0);   // Duty Cycle Percent / Load Cell Force (%/lb)

EngineSpeed engineSpeed(4);
WheelSpeed rWheelsSpeed(24);
// WheelSpeed flWheelSpeed(24);
// WheelSpeed frWheelSpeed(24);

Encoder pEnc(P_ENC_A, P_ENC_B); // 1 Encoder Count = ~1/3606 Revolution
Encoder sEnc(S_ENC_A, S_ENC_B); // 1 Encoder Count = ~1/3606 Revolution

LoadCell pLC(0, 0, 0); // Load Cell Force = SHIFTLINK_TOP/SHIFTLINK_ALL * Clamping Force
LoadCell sLC(0, 0, 0); // Load Cell Force = SHIFTLINK_TOP/SHIFTLINK_ALL * Clamping Force

// BrakePressure fBrakePressure(FBRAKE_PRESSURE);
// BrakePressure rBrakePressure(RBRAKE_PRESSURE);

Motor pMot(P_MOT_INA, P_MOT_INB, P_MOT_PWM);
Motor sMot(S_MOT_INA, S_MOT_INB, S_MOT_PWM);

IntervalTimer ctrlTimer;
IntervalTimer commTimer;
const uint32_t CTRL_PERIOD = 10000; // Microseconds (us)
const uint32_t COMM_PERIOD = 10000; // Microseconds (us)

/* ** FINITE STATE MACHINES ** */

FSMVars fsm;

Engine engine(fsm, ePID);
Primary primary(fsm, pEncPID, pEnc, pLC, pMot);
Secondary secondary(fsm, sEncPID, sLcPID, sEnc, sLC, sMot);
HallEffectTask hallEffectTask(fsm, engineSpeed, rWheelsSpeed);
// PressureTransducerTask pressureTransducerTask(fsm, fBrakePressure, rBrakePressure);
LaunchControl launchControl(fsm, LAUNCH_BUTTON);
DashboardLEDs dashboardLEDs(fsm, UPSHIFT_LED, BKSHIFT_LED);
Communication communication(fsm, engine, primary, secondary);

/* ** INTERRUPT SERVICE ROUTINES ** */

void engineSpeedISR();
void rWheelsSpeedISR();
void ctrlISR();
void commISR();

/* ** MAIN ** */

void setup()
{

// Serial Monitor
#ifdef INFO
    Serial.begin(9600);
    while (!Serial) // Wait for serial port to connect. Needed for native USB only.
    {
    }
#endif

// Bench Testing
#ifdef INFO
    Serial.println("Connect the motor wires!");
    Serial.println("Delaying for 2 seconds..");
    delay(2000);
    Serial.println("GO!");
#endif

    // Hall Effect Sensor Setup
    pinMode(ENGINE_SPEED_PIN, INPUT);
    pinMode(RWHEELS_SPEED_PIN, INPUT);
    // pinMode(FLWHEEL_SPEED_PIN, INPUT);
    // pinMode(FRWHEEL_SPEED_PIN, INPUT);

    // Encoder Setup
    /* Handled by Encoder constructor! */

    // Load Cell Setup
    /* Handled by LoadCell begin() function! */

    // Brake Pressure Setup
    // TODO: Move to BrakePressure constructor
    // pinMode(FBRAKE_PRESSURE, INPUT);
    // pinMode(RBRAKE_PRESSURE, INPUT);

    // Motor Setup
    /* Handled by Motor begin() function! */

    // Dashboard Setup
    // TODO: Move to Dashboard FSM
    pinMode(LAUNCH_BUTTON, INPUT_PULLUP);
    pinMode(UPSHIFT_LED, OUTPUT);
    pinMode(BKSHIFT_LED, OUTPUT);

    // Pin Interrupt Setup
    attachInterrupt(digitalPinToInterrupt(ENGINE_SPEED_PIN), engineSpeedISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RWHEELS_SPEED_PIN), rWheelsSpeedISR, RISING);
    // attachInterrupt(digitalPinToInterrupt(FLWHEEL_SPEED_PIN), flWheelSpeedISR, RISING);
    // attachInterrupt(digitalPinToInterrupt(FRWHEEL_SPEED_PIN), frWheelSpeedISR, RISING);

    // Timer Interrupt Setup
    commTimer.begin(commISR, COMM_PERIOD);
    ctrlTimer.begin(ctrlISR, CTRL_PERIOD);

    // FSM Variables Setup
    // TODO: Move to FSMVars
    fsm.run = true;
    fsm.eCalc = false;
    fsm.pCalc = false;
    fsm.sCalc = false;
    fsm.comm = false;
    fsm.eSpeed = 0;         // Revolutions per Minute (RPM)
    fsm.rwSpeed = 0;        // Revolutions per Minute (RPM)
    fsm.fBrakePressure = 0; // 13-bit ADC (0-8191)
    fsm.rBrakePressure = 0; // 13-bit ADC (0-8191)
    fsm.engaged = false;
    fsm.ePIDOutput = 0;
    fsm.pPIDOutput = 0;
    fsm.sPIDOutput = 0;
}

void loop()
{
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
    // pressureTransducerTask.run();
    // launchControl.run();
    // ecvtstatusLED.run();
    // dashboardLEDs.run();
    // communication.run();
}

/* **INTERRUPT SERVICE ROUTINES** */

// Hall Effect Sensors
void engineSpeedISR() { engineSpeed.calc(); }
void rWheelsSpeedISR() { rWheelsSpeed.calc(); }
// void flWheelSpeedISR() { flWheelSpeed.calc(); }
// void frWheelSpeedISR() { frWheelSpeed.calc(); }

// Timers
void ctrlISR()
{
    fsm.eCalc = true;
    fsm.pCalc = true;
    fsm.sCalc = true;
}

void commISR() { fsm.comm = true; }
