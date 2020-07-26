/*
 * Communication.h - DAQ and eCVT communication configuration.
 * Created by Rahul Goyal, December 1, 2019.
 * Released to Cal Poly Baja SAE. ;)
 */

#ifndef Communication_h
#define Communication_h

#include <Arduino.h>

// Communication
const int8_t  ECVT_DATA_SIZE = 37;		// Bytes
const int8_t START_DATA_SIZE = 2;		// Bytes
const int8_t CHECK_DATA_SIZE = 2;		// Bytes
const int8_t START_BYTE_VAL  = 0xAA;    // 1010 1010

#endif