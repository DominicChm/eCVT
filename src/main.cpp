/*
 * main.cpp - Main code to control eCVT.
 * Created by Rahul Goyal, July 2019.
 * Maintained by Rahul Goyal, 2019-21.
 * Maintained by Shaina Bagri, 2020-21.
 * Released to Cal Poly Baja SAE. ;)
 */

/* Note: TEST AND DEBUG depend on INFO */
#define INFO
// #define TEST
// #define DEBUG

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
#include "./LoadCellTask/LoadCellTask.h"
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
    - The  integral  gain (Ki) must decrease linearly with CTRL_PERIOD.
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

int32_t LC_CALIBRATION = 10056;                   // HX711 Unit / Load Cell Force (#/lb)
LoadCell pLC(P_LC_SCK, P_LC_SDA, LC_CALIBRATION); // Load Cell Force = SHIFTLINK_TOP/SHIFTLINK_ALL * Clamping Force
LoadCell sLC(S_LC_SCK, S_LC_SDA, LC_CALIBRATION); // Load Cell Force = SHIFTLINK_TOP/SHIFTLINK_ALL * Clamping Force

// BrakePressure fBrakePressure(FBRAKE_PRESSURE);
// BrakePressure rBrakePressure(RBRAKE_PRESSURE);

Motor pMot(P_MOT_INA, P_MOT_INB, P_MOT_PWM);
Motor sMot(S_MOT_INA, S_MOT_INB, S_MOT_PWM);

IntervalTimer ctrlTimer;
IntervalTimer commTimer;
const uint32_t CTRL_PERIOD = 10000; // Microseconds (us)
const uint32_t COMM_PERIOD = 1000000; // Microseconds (us)

/* ** FINITE STATE MACHINES ** */

FSMVars fsm;

Engine engine(fsm, ePID);
Primary primary(fsm, pEnc, pMot, pEncPID);
Secondary secondary(fsm, sEnc, sMot, sEncPID, sLcPID);
HallEffectTask hallEffectTask(fsm, engineSpeed, rWheelsSpeed);
LoadCellTask loadCellTask(fsm, pLC, sLC);
// PressureTransducerTask pressureTransducerTask(fsm, fBrakePressure, rBrakePressure);
LaunchControl launchControl(fsm, LAUNCH_BUTTON);
DashboardLEDs dashboardLEDs(fsm, UPSHIFT_LED, BKSHIFT_LED);
Communication communication(fsm, engine, primary, secondary);

/* ** INTERRUPT SERVICE ROUTINES ** */

// Hall Effect Sensors
void engineSpeedISR() { hallEffectTask.engineSpeedISR(); }
void rWheelsSpeedISR() { hallEffectTask.rWheelsSpeedISR(); }
// void flWheelSpeedISR() { flWheelSpeed.calc(); }
// void frWheelSpeedISR() { frWheelSpeed.calc(); }

// Control Loop Timer
void ctrlISR()
{
    fsm.eCalc = true;
    fsm.pCalc = true;
    fsm.sCalc = true;
}

// Communication Timer
void commISR() { fsm.comm = true; }

/* ** TEST ** */

#ifdef TEST
/* Tests motor/encoder direction and load cell readings. */
void runTest()
{
    Serial.println();
    Serial.println("Motor/Encoder Test started.");

    pMot.begin();
    sMot.begin();

    pEnc.write(0);
    sEnc.write(0);
    pMot.setDutyCycle(-10);
    sMot.setDutyCycle(-10);

    delay(1000);

    pMot.setDutyCycle(0);
    sMot.setDutyCycle(0);

    Serial.println("Motor/Encoder Test finished.");

    Serial.println();
    Serial.print(" Primary  Encoder: ");
    Serial.println(pEnc.read());
    Serial.print("Secondary Encoder: ");
    Serial.println(sEnc.read());

    Serial.println();
    Serial.println("Both clutch sheaves should have opened.");
    Serial.println("Both encoder values should be negative.");

    Serial.println();
    Serial.println("Tip: first fix motor direction, then fix encoder direction.");

    delay(1000);

    Serial.println();
    Serial.println("Load Cell Test started.");

    pLC.begin();
    sLC.begin();

    while (true)
    {
        Serial.println();
        Serial.print(" Primary  Load Cell: ");
        Serial.println(pLC.read());
        Serial.print("Secondary Load Cell: ");
        Serial.println(sLC.read());

        delay(250);
    }
}
#endif

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

#ifdef TEST
    runTest();
#endif
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
    loadCellTask.run();

    // Bonus Tasks
    // pressureTransducerTask.run();
    // launchControl.run();
    // ecvtstatusLED.run();
    // dashboardLEDs.run();
    communication.run();
}
