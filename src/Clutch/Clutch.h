#ifndef Clutch_h
#define Clutch_h

#include "./Task/Task.h"
#include "Motor.h"
#include <Encoder.h>
#include "LoadCell.h"
#include "LookupTables.h"

class Clutch : public Task
{
public:
    enum State
    {
        INITIALIZE,
        CALIBRATE_OPEN_SHEAVES,
        CALIBRATE_ZERO_ENCODER,
        CALIBRATE_WAIT_USER,
        CONTROLLER_REST,
        CONTROLLER_UPDATE
    };

    Clutch(
        FSMVars fsm,
        Encoder enc,
        Motor mot);

    void run();
    int8_t getState();
    Encoder getEnc();

protected:
    const int8_t MAX_STATIC_DUTYCYCLE = 25; // Magnitude of Duty Cycle Percent (%)

    Encoder enc;

    virtual bool getCalc() = 0;
    virtual void initController() = 0;
    virtual void updateController() = 0;

    // TODO: Refactor
    virtual int16_t readLoadCell() = 0;

    void setMotorDutyCycle(int16_t dutyCycle);

private:
    State state = INITIALIZE;
    Motor mot;

    uint32_t calTime; // Milliseconds (ms)
};

#endif
