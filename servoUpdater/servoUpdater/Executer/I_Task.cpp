// 
// 
// 

#include "I_Task.h"

void I_Task::configTask(String _task_ID, uint16_t _period_ms, uint8_t _priority){
	task_ID = _task_ID;
	executionPeriod = _period_ms;
	priority = _priority;
	lastTimeExecutedMillis = millis();
}

uint16_t I_Task::getExecutionPeriod(){
	
	return executionPeriod;
}


uint16_t I_Task::getLastMillisExecuted(){
	
	return lastTimeExecutedMillis;
}

bool I_Task::getExecutionFlag(){
	
	uint16_t currentMillis = millis();
	if ((currentMillis - lastTimeExecutedMillis) > executionPeriod)
		return true;
	else
		return false;
}

void I_Task::update(){
	
	
}