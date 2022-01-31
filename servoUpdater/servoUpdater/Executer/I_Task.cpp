// 
// 
// 

#include "I_Task.h"

void I_Task::setExecutionPeriod(execType _timerType, unsigned int _period){
	
	timerType = _timerType;
	executionPeriod = _period;
	lastTimeExecuted = millis();
}

unsigned int I_Task::getExecutionPeriod(){
	return executionPeriod;
}

unsigned long I_Task::getLastMillisExecuted(){
	return lastTimeExecuted;
}

bool I_Task::getExecutionFlag(){
	bool return_val;
	
	switch (timerType) {
		case execType::inMillis:
			currentMillis = millis();
			if (abs(currentMillis - lastTimeExecuted) >= executionPeriod) {
				lastTimeExecuted = currentMillis;
				return_val = true;
			}
			else return_val = false;
			break;
			
		case execType::inMicros:
			currentMicros = micros();
			if (abs(currentMicros - lastTimeExecuted) >= executionPeriod) {
				lastTimeExecuted = currentMicros;
				return_val = true;
			}
			else return_val = false;
			break;
	}
	
	return return_val;
}