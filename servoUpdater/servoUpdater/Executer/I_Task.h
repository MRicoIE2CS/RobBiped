// Task.h

#ifndef _TASK_h
#define _TASK_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif



class I_Task{
	
	public:
		
		enum class execType { inMillis, inMicros };
		void setExecutionPeriod(execType _timerType, unsigned int _period);
		
		unsigned int getExecutionPeriod();
		unsigned long getLastMillisExecuted();
		bool getExecutionFlag();
		
		
		void update();
		
	protected:
	
	
		execType timerType = execType::inMillis;
		unsigned int executionPeriod;
		unsigned long lastTimeExecuted;
		
		unsigned long currentMillis;
		unsigned long currentMicros;
	
	};

#endif

