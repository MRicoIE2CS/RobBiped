// Task.h

#ifndef _TASK_h
#define _TASK_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif



class I_Task{
	
	protected:
		
		String task_ID;		// useful?
		uint16_t executionPeriod;
		uint8_t priority;	// useful?
		uint16_t lastTimeExecutedMillis;
		
	public:
	
		void configTask(String _task_ID, uint16_t _period_ms, uint8_t _priority);
		
		uint16_t getExecutionPeriod();
		uint16_t getLastMillisExecuted();
		bool getExecutionFlag();
		
		
		void update();
	
	};

#endif

