/*
 * eCVT.ino - Main code to operate DAQ.
 * Created by Rahul Goyal, January 1, 2020.
 * Released to Cal Poly Baja SAE. ;)
 */

#define DEBUG
#define TEST

#include <Arduino.h>
#include <Bounce2.h>
#include <SD.h>
#include "EngineSpeed.h"
#include "WheelSpeed.h"



/* ** CONFIG ** */

#include "WiringDec2019.h"
// #include "WiringMar2019.h"
#include "Communication.h"



/* ** SYSTEM ** */

// Hall Effect Sensors
EngineSpeed engineSpeed( 8);
WheelSpeed  eWheelSpeed( 8);
WheelSpeed rWheelsSpeed(24);
WheelSpeed flWheelSpeed(24);
WheelSpeed frWheelSpeed(24);

// Debounced Buttons
Bounce loggerButton;

// Filename Base
const char FILENAME_BASE[] = "TEST_";

// Dashboard LED
const uint32_t FLASH_PERIOD = 500;		// Milliseconds (ms)

// Structure of Data
struct Data {
	uint32_t relTime;
	int16_t  eSpeed;
	int16_t ewSpeed;
	int16_t rwSpeed;
	int16_t flSpeed;
	int16_t frSpeed;
	int16_t fPressure;
	int16_t rPressure;
	bool markerButton;
	uint8_t eCVT[ECVT_DATA_SIZE];
} data;

// Timers
IntervalTimer writeTimer;
IntervalTimer flushTimer;
const uint32_t WRITE_PERIOD =     1000;	// Microseconds (us)
const uint32_t FLUSH_PERIOD = 60000000;	// Microseconds (us)

#ifdef TEST
IntervalTimer  testTimer;
const uint32_t TEST_PERIOD  =  5000000; // Microseconds (us)
#endif



/* ** FINITE STATE MACHINE ** */

// Inter-Communication Variables
volatile bool write = false;
volatile bool flush = false;
#ifdef TEST
volatile bool test  =  true;
#else
volatile bool test  = false;
#endif

/*	The statuses are:
		Status 0 - No change.
		Status 1 - Change to off.
		Status 2 - Change to on.
		Status 3 - Error.
*/
int8_t status = 0;



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
	pinMode(DAQ_STATUS_LED, OUTPUT);
}

void loop() {
	// Debugging
	#ifdef DEBUG
	// Serial.println();
	delay(10);
	#endif

	// Tasks
	daq();
	statusLED();
	// communication();
}



/* **TASKS** */

void daq() {

	static int8_t state = 0;

	// Debugging
	#ifdef DEBUG
	// Serial.print("state: ");
	// Serial.println(state);
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

	switch (state) {

		// INITIALIZE
		case 0:
			// Logger Button
			loggerButton = Bounce();
			loggerButton.attach(LOGGER_BUTTON);
			loggerButton.interval(10);	// Milliseconds (ms)

			// Timer Interrupt Setup
			writeTimer.begin(writeISR, WRITE_PERIOD);
			flushTimer.begin(flushISR, FLUSH_PERIOD);
			#ifdef TEST
			 testTimer.begin( testISR,  TEST_PERIOD);
			#endif

			// SD Card and State Changes
			if (SD.begin(BUILTIN_SDCARD)) {
				state = 1;
			} else {
				status = 3;
				state = 9;
			}
			return;

		// IDLE STATE
		case 1:
			// State Changes
			if (loggerButton.fell() || test) {
				state = 2;
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
				state = 3;
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
				status = 2;
				state = 4;
			} else {
				status = 3;
				state = 9;
			}
			return;

		// LOG DATA - REST
		case 4:
			// State Changes
			if (loggerButton.fell() || test) {
				state = 7;
			} else if (flush) {
				state = 6;
			} else if (write) {
				state = 5;
			}

			// Reset flag
			#ifdef TEST
			test = false;
			#endif

			return;

		// WRITE DATA
		case 5:
			// Store relative time
			data.relTime = micros() - calTime;
			Serial.println(data.relTime);

			// Store hall effect sensor data
			noInterrupts();
			data.eSpeed =  engineSpeed.read();
			  interrupts();
			noInterrupts();
			data.ewSpeed =  eWheelSpeed.read();
			  interrupts();
			noInterrupts();
			data.rwSpeed = rWheelsSpeed.read();
			  interrupts();
			noInterrupts();
			data.flSpeed = flWheelSpeed.read();
			  interrupts();
			noInterrupts();
			data.frSpeed = frWheelSpeed.read();
			  interrupts();

			// Store pressure transducer data
			data.fPressure = analogRead(FBRAKE_PRESSURE);
			data.rPressure = analogRead(RBRAKE_PRESSURE);

			// Store marker button status
			data.markerButton = digitalRead(MARKER_BUTTON);

			// Write all data
			file.write((const uint8_t *)&data, sizeof(data));

			// Reset flag
			write = false;

			// State Changes
			state = 4;
			return;

		// FLUSH DATA
		case 6:
			// Flush data
			file.flush();

			// Reset flag
			flush = false;

			// State Changes
			state = 4;
			return;

		// CLOSE FILE
		case 7:
			// Close file
			file.close();

			// State Changes
			status = 1;
			state = 1;
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



void statusLED() {

	static int8_t state = 0;

	switch (state) {

		// INITIALIZE
		case 0:
			state = 1;
			return;

		// HUB STATE
		case 1:
			// State Changes
			if (status == 0) {
				return;
			} else if (status == 1) {
				state = 2;
			} else if (status == 2) {
				state = 3;
			} else if (status == 3) {
				state = 4;
			}
			return;

		// TURN LED OFF
		case 2:
			digitalWrite(DAQ_STATUS_LED, LOW);

			// State Changes
			status = 0;
			state = 1;
			return;

		// TURN LED ON
		case 3:
			digitalWrite(DAQ_STATUS_LED, HIGH);

			// State Changes
			status = 0;
			state = 1;
			return;

		// FLASH (OFF)
		case 4:
			return;

		// FLASH (ON)
		case 5:
			return;
	}

}



void communication() {

	static int8_t state = 0;

	static uint8_t numBytesRead = 0;
	static uint8_t buffer[ECVT_DATA_SIZE];

	static uint16_t  daqChecksum;
	static uint16_t ecvtChecksum;

	switch (state) {

		// INITIALIZE
		case 0:
			state = 1;
			return;

		// READ START DATA
		case 1:
			// Wait until byte available
			if (!Serial.available()) {
				return;
			}

			// If byte read is start byte...
			if (DAQ_ECVT_SERIAL.read() == START_BYTE_VAL) {
				// Increment number of bytes read
				numBytesRead++;
			// Otherwise...
			} else {
				// Reset number of bytes read
				numBytesRead = 0;
			}

			// State Changes
			if (numBytesRead >= START_DATA_SIZE) {
				daqChecksum = 0;
				state = 2;
			}

		// READ ECVT DATA
		case 2:
			// Wait until byte available
			if (!Serial.available()) {
				return;
			}

			// Read byte and store in buffer
			uint8_t read;
			read = DAQ_ECVT_SERIAL.read();
			buffer[numBytesRead - START_DATA_SIZE] = read;
			daqChecksum += numBytesRead * read;
			numBytesRead++;

			// State Changes
			if (numBytesRead >= ECVT_DATA_SIZE) {
				state = 3;
			}
			return;

		// READ CHECK DATA
		case 3:
			// Wait until bytes available
			if (!(Serial.available() >= 2)) {
				return;
			}

			// Read bytes and store in checksum
			ecvtChecksum = ((uint16_t)Serial.read() << 8) + Serial.read();
			numBytesRead += CHECK_DATA_SIZE;

			// State Changes
			state = 4;
			return;

		// VERIFY DATA INTEGRITY
		case 4:
			// State Changes
			if (daqChecksum == ecvtChecksum) {
				state = 5;
			} else {
				// Reset number of bytes read and change state
				numBytesRead = 0;
				state = 1;
			}
			return;

		// COPY BUFFER TO ECVT DATA
		case 5:
			// Copy buffer to eCVT data
			memcpy(data.eCVT, buffer, ECVT_DATA_SIZE);

			// State Changes
			// Reset number of bytes read and change state
			numBytesRead = 0;
			state = 1;
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
