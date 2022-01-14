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
    volatile bool run;
    volatile bool eCalc;
    volatile bool pCalc;
    volatile bool sCalc;
    volatile bool comm;
    // Sensors
    int16_t eSpeed;
    int16_t rwSpeed;
    int16_t fBrakePressure;
    int16_t rBrakePressure;
    // Control
    bool engaged;
    int16_t ePIDOutput; // Ratio Percent
    int16_t pPIDOutput; // Duty Cycle Percent
    int16_t sPIDOutput; // Duty Cycle Percent
};

#endif
