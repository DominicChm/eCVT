/*
 * eCVT.ino - Main code to operate DAQ.
 * Created by Rahul Goyal, January 1, 2020.
 * Released to Cal Poly Baja SAE. ;)
 */

#define DEBUG 1
#define TEST  1

#include <Arduino.h>
#include <Bounce2.h>
#include <SD.h>
#include "EngineSpeed.h"
#include "WheelSpeed.h"

#define hiByte highByte
#define loByte  lowByte



/* ** WIRING ** */

// Hall Effect Sensors
const int8_t  ENGINE_SPEED_PIN =  5;
const int8_t RWHEELS_SPEED_PIN =  6;
const int8_t FLWHEEL_SPEED_PIN = 29;
const int8_t FRWHEEL_SPEED_PIN = 30;

// Pressure Transducers
const int8_t FBRAKE_PRESSURE = 34;
const int8_t RBRAKE_PRESSURE = 33;

// Dashboard
const int8_t LOGGER_BUTTON = 2;
const int8_t MARKER_BUTTON = 3;
const int8_t STATUS_LED = 4;

// Communication
#define ECVT_SERIAL Serial1
#define XBEE_SERIAL Serial2



/* ** SYSTEM ** */

// Hall Effect Sensors
EngineSpeed engineSpeed( 8);
WheelSpeed  eWheelSpeed( 8);
WheelSpeed rWheelsSpeed(24);
WheelSpeed flWheelSpeed(24);
WheelSpeed frWheelSpeed(24);

// Debounced Buttons
Bounce loggerButton = Bounce();

// Filename Base
const char FILENAME_BASE[] = "TEST_";

// Communication
const int8_t ECVT_DATA_SIZE = 33;		// Bytes
const int8_t START_DATA_SIZE = 2;		// Bytes
const int8_t CHECK_DATA_SIZE = 2;		// Bytes
const int16_t START_BYTE_VAL = 0x5555;	// 0101 0101

// Timers
IntervalTimer writeTimer;
IntervalTimer flushTimer;
const uint32_t WRITE_PERIOD =     1000;	// Microseconds (us)
const uint32_t FLUSH_PERIOD = 10000000;	// Microseconds (us)

#ifdef TEST
IntervalTimer  testTimer;
const uint32_t TEST_PERIOD  =  5000000; // Microseconds (us)
#endif



/* ** FINITE STATE MACHINE ** */

// States
int8_t dState = 0;
int8_t cState = 0;

// Inter-Communication Variables
bool write = false;
bool flush = false;
#ifdef TEST
bool test  =  true;
#else
bool test  = false;
#endif
uint8_t ecvtData[ECVT_DATA_SIZE];



/* ** MAIN ** */

void setup() {
	// Serial Monitor
	#ifdef DEBUG
	Serial.begin(2000000);
	while (!Serial) { }	// Wait for serial port to connect. Needed for native USB only.
	#endif

	// Hall Effect Sensor Setup
	pinMode( ENGINE_SPEED_PIN, INPUT);
	pinMode(RWHEELS_SPEED_PIN, INPUT);
	pinMode(FLWHEEL_SPEED_PIN, INPUT);
	pinMode(FRWHEEL_SPEED_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt( ENGINE_SPEED_PIN),  engineSpeedISR, RISING);
	attachInterrupt(digitalPinToInterrupt(RWHEELS_SPEED_PIN), rWheelsSpeedISR, RISING);
	attachInterrupt(digitalPinToInterrupt(FLWHEEL_SPEED_PIN), flWheelSpeedISR, RISING);
	attachInterrupt(digitalPinToInterrupt(FRWHEEL_SPEED_PIN), frWheelSpeedISR, RISING);

	// Pressure Transducer Setup
	pinMode(FBRAKE_PRESSURE, INPUT);
	pinMode(RBRAKE_PRESSURE, INPUT);

	// Dashboard Setup
	pinMode(LOGGER_BUTTON, INPUT_PULLUP);
	pinMode(MARKER_BUTTON, INPUT_PULLUP);
	pinMode(STATUS_LED, OUTPUT);

	// Timer Interrupt Setup
	writeTimer.begin(writeISR, WRITE_PERIOD);
	flushTimer.begin(flushISR, FLUSH_PERIOD);
	#ifdef TEST
	 testTimer.begin( testISR,  TEST_PERIOD);
	#endif
}

void loop() {
	// Debugging
	#ifdef DEBUG
	// Serial.println();
	delay(10);
	#endif

	// Tasks
	daq();
	// comm();
}



/* **TASKS** */

void daq() {

	// Debugging
	#ifdef DEBUG
	// Serial.print("dState: ");
	// Serial.println(dState);
	#endif

	static uint32_t calTime;

	// Create filename character array
	/* The size is:
			The size of FILENAME_BASE (including the null terminator)
			+5 for 16-bit num in decimal representation
			+4 for the file extension
			+1 for safety.
	*/
	static char filename[sizeof(FILENAME_BASE) + 10];

	static int16_t num = 1;
	static File file;

	switch (dState) {

		// INITIALIZE
		case 0:
			// Logger Button
			loggerButton.attach(LOGGER_BUTTON);
			loggerButton.interval(10);	// Milliseconds (ms)

			// SD Card and State Changes
			if (SD.begin(BUILTIN_SDCARD)) {
				dState = 1;
			} else {
				dState = 9;
			}
			return;

		// IDLE STATE
		case 1:
			// State Changes
			if (loggerButton.fell() || test) {
				dState = 2;
			}

			// Reset flag
			#ifdef TEST
			test = false;
			#endif
			
			return;

		// NAME FILE
		case 2:
			// Store number as character array
			char numArray[5];
			sprintf(numArray, "%d", num);

			// Copy base filename and concatenate number and file extension
			strcpy(filename, FILENAME_BASE);
			strcat(filename, numArray);
			strcat(filename, ".log");

			// State Changes
			/* If filename exists, increment number and try again next loop.
			   Else (if filename does not exist), change state to open file. */
			if (SD.exists(filename)) {
				num++;
			} else {
				dState = 3;
			}
			return;

		// OPEN FILE
		case 3:
			// Open file
			file = SD.open(filename, FILE_WRITE);

			// State Changes
			if (file) {
				// Set calibration time and change state
				calTime = micros();
				dState = 4;
			} else {
				dState = 9;
			}
			return;

		// LOG DATA - REST
		case 4:
			// State Changes
			if (loggerButton.fell() || test) {
				dState = 7;
			} else if (flush) {
				dState = 6;
			} else if (write) {
				dState = 5;
			}

			// Reset flag
			#ifdef TEST
			test = false;
			#endif

			return;

		// WRITE DATA
		case 5:
		{
			// Store relative time
			uint32_t relTime = micros() - calTime;
			Serial.println(relTime);

			// Store hall effect sensor data
			noInterrupts();
			int16_t  eSpeed =  engineSpeed.read();
			  interrupts();
			noInterrupts();
			int16_t ewSpeed =  eWheelSpeed.read();
			  interrupts();
			noInterrupts();
			int16_t rwSpeed = rWheelsSpeed.read();
			  interrupts();
			noInterrupts();
			int16_t flSpeed = flWheelSpeed.read();
			  interrupts();
			noInterrupts();
			int16_t frSpeed = frWheelSpeed.read();
			  interrupts();

			// Store pressure transducer data
			int16_t fPressure = analogRead(FBRAKE_PRESSURE);
			int16_t rPressure = analogRead(RBRAKE_PRESSURE);

			// Write relative time
			file.write(relTime >> 24);
			file.write(relTime >> 16);
			file.write(relTime >>  8);
			file.write(relTime >>  0);

			// Write hall effect sensor data
			file.write(hiByte( eSpeed));
			file.write(loByte( eSpeed));
			file.write(hiByte(ewSpeed));
			file.write(loByte(ewSpeed));
			file.write(hiByte(rwSpeed));
			file.write(loByte(rwSpeed));
			file.write(hiByte(flSpeed));
			file.write(loByte(flSpeed));
			file.write(hiByte(frSpeed));
			file.write(loByte(frSpeed));

			// Write pressure transducer data
			file.write(hiByte(fPressure));
			file.write(loByte(fPressure));
			file.write(hiByte(rPressure));
			file.write(loByte(rPressure));

			// Write new line character
			file.println();

			// Reset flag
			write = false;

			// State Changes
			dState = 4;
			return;
		}

		// FLUSH DATA
		case 6:
			// Flush data
			file.flush();

			// Reset flag
			flush = false;

			// State Changes
			dState = 4;
			return;

		// CLOSE FILE
		case 7:
			// Close file
			file.close();

			// State Changes
			dState = 1;
			return;

		// SD ERROR
		// TODO
		case 9:
			#ifdef DEBUG
			Serial.println("SD Error!!");
			#endif
			while (true) { }
			return;
	}
}



void comm() {

	static uint8_t numBytesRead = 0;
	static uint8_t buffer[ECVT_DATA_SIZE];
	static uint16_t daqChecksum;
	static uint16_t	ecvtChecksum;

	switch (cState) {

		// INITIALIZE
		case 0:
			cState = 1;
			return;

		// READ START DATA
		case 1:
			// Wait until byte available
			if (!Serial.available()) {
				return;
			}

			// If byte read is start byte...
			if (ECVT_SERIAL.read() == START_BYTE_VAL) {
				// Increment number of bytes read
				numBytesRead++;
			// Otherwise...
			} else {
				// Reset number of bytes read
				numBytesRead = 0;
			}

			// State Changes
			if (numBytesRead >= START_DATA_SIZE) {
				cState = 2;
			}

		// READ ECVT DATA
		case 2:
			// Wait until byte available
			if (!Serial.available()) {
				return;
			}

			// Read byte and store in buffer
			buffer[numBytesRead - START_DATA_SIZE] = ECVT_SERIAL.read();
			numBytesRead++;

			// State Changes
			if (numBytesRead >= ECVT_DATA_SIZE) {
				cState = 3;
			}
			return;

		// GENERATE CHECKSUM
		case 3:
			// Generate checksum
			daqChecksum = CRC16(ecvtData);

			// State Changes
			cState = 4;
			return;

		// READ CHECK DATA
		case 4:
			// Wait until bytes available
			if (!(Serial.available() >= 2)) {
				return;
			}

			// Read bytes and store in checksum
			ecvtChecksum = ((uint16_t)Serial.read() << 8) + Serial.read();

			// State Changes
			cState = 5;
			return;

		// VERIFY DATA INTEGRITY
		case 5:
			// State Changes
			if (daqChecksum == ecvtChecksum) {
				cState = 6;
			} else {
				cState = 1;
			}
			return;

		// COPY BUFFER TO ECVT DATA
		case 6:
			// Copy buffer to eCVT data
			memcpy(ecvtData, buffer, ECVT_DATA_SIZE);

			// State Changes
			cState = 1;
			return;
	}
}



/* **INTERRUPT SERVICE ROUTINES** */

// Hall Effect Sensors
void  engineSpeedISR() { 
	engineSpeed.calc();
	eWheelSpeed.calc();
}
void rWheelsSpeedISR() { rWheelsSpeed.calc(); }
void flWheelSpeedISR() { flWheelSpeed.calc(); }
void frWheelSpeedISR() { frWheelSpeed.calc(); }

// Timers
void writeISR() { write = true; }
void flushISR() { flush = true; }
#ifdef TEST
void  testISR() { 
	test = !test;
}
#endif



/* **CRC-16 CHECKSUM ALGORITHM** */

int16_t CRC16(uint8_t* buffer) {
	return 1;
}