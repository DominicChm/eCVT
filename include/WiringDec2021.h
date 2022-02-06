/*
 *  WiringDec2019.h - DAQ and eCVT wiring configuration.
 *  Created by Rahul Goyal, December 2019.
 *  Released to Cal Poly Baja SAE. ;)
 */

#ifndef WiringDec2021_h
#define WiringDec2021_h

#include <Arduino.h>

// Hall Effect Sensors
const int8_t ENGINE_SPEED_PIN = 5;
const int8_t RWHEELS_SPEED_PIN = 6;

// Encoders
/* Swap A and B pins to swap direction. */
const int8_t P_ENC_A = 24;
const int8_t P_ENC_B = 25;
const int8_t S_ENC_A = 27;
const int8_t S_ENC_B = 26;

// Load Cells
const int8_t P_LC_SCK = 13;
const int8_t P_LC_SDA = 28;
const int8_t S_LC_SCK = 14;
const int8_t S_LC_SDA = 15;

// Motors
/* Swap A and B pins to swap direction. */
const int8_t P_MOT_INA = 37;
const int8_t P_MOT_INB = 36;
const int8_t P_MOT_PWM = 22;
const int8_t S_MOT_INA = 20;
const int8_t S_MOT_INB = 21;
const int8_t S_MOT_PWM = 23;

// Pressure Transducers
// const int8_t FBRAKE_PRESSURE = 34;
// const int8_t RBRAKE_PRESSURE = 33;

// Dashboard
const int8_t LAUNCH_BUTTON = 3;
const int8_t ECVT_STATUS_LED = 2;
const int8_t UPSHIFT_LED = 0;
const int8_t BKSHIFT_LED = 1;

// Communication
#define DAQ_ECVT_SERIAL Serial2

#endif
