#include "HallEffectTask.h"


HallEffectTask::HallEffectTask(FSMVars fsm, EngineSpeed engineSpeed, WheelSpeed rWheelsSpeed):
    Task(fsm), engineSpeed(engineSpeed), rWheelsSpeed(rWheelsSpeed) {}


void HallEffectTask::run() {
    switch(state) {
        case INITIALIZE:
            state = UPDATE;
            return;

        case UPDATE:
            cli();
	    	fsm.eSpeed  =  engineSpeed.read();
	    	sei();

            cli();
	    	fsm.rwSpeed = rWheelsSpeed.read();
	    	sei();

            // cli();
	    	// flSpeed = flWheelSpeed.read();
	    	// sei();

            // cli();
	    	// frSpeed = frWheelSpeed.read();
	    	// sei();

            return;
    }
}