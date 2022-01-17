/*
 *	FSMVars.h - Data transfer object header for FSM variables.
 *	Created by Shaina Bagri, January 2021.
 *	Released to Cal Poly Baja SAE. ;)
 */

#ifndef FSMVars_h
#define FSMVars_h

#include <Arduino.h>

class FSMVars
{
public:
    FSMVars();

    // Interrupts
    volatile bool run = true;
    volatile bool eCalc = false;
    volatile bool pCalc = false;
    volatile bool sCalc = false;
    volatile bool comm = false;
    // Sensors
    int16_t eSpeed = 0;         // Revolutions Per Minute (RPM)
    int16_t rwSpeed = 0;        // Revolutions Per Minute (RPM)
    int16_t pLoadCellForce = 0; // Load Cell Force (lb)
    int16_t sLoadCellForce = 0; // Load Cell Force (lb)
    int16_t fBrakePressure = 0; // Brake Pressure (psi)
    int16_t rBrakePressure = 0; // Brake Pressure (psi)
    // Control
    bool engaged = false;
    int16_t ePIDOutput = 0; // Ratio Percent
    int16_t pPIDOutput = 0; // Duty Cycle Percent
    int16_t sPIDOutput = 0; // Duty Cycle Percent
};

#endif
