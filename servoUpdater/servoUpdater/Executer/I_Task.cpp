// 
// 
// 

#include "I_Task.h"

void I_Task::configTask(String _task_ID, unsigned int _period_ms, unsigned short _priority){
	
	task_ID = _task_ID;
	executionPeriod = _period_ms;
	priority = _priority;
	lastTimeExecutedMillis = millis();
}

unsigned int I_Task::getExecutionPeriod(){
	
	return executionPeriod;
}

unsigned long I_Task::getLastMillisExecuted(){
	
	return lastTimeExecutedMillis;
}

bool I_Task::getExecutionFlag(){
	
	unsigned long currentMillis = millis();
	if (abs(currentMillis - lastTimeExecutedMillis) >= executionPeriod) {
		lastTimeExecutedMillis = currentMillis;
		return true;
	}
	else return false;
}