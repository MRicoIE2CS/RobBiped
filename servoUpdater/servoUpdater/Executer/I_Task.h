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
		unsigned int executionPeriod;
		unsigned short priority;	// useful?
		unsigned long lastTimeExecutedMillis;
		
	public:
	
		void configTask(String _task_ID, unsigned int _period_ms, unsigned short _priority);
		
		unsigned int getExecutionPeriod();
		unsigned long getLastMillisExecuted();
		bool getExecutionFlag();
		
		
		void update();
	
	};

#endif

