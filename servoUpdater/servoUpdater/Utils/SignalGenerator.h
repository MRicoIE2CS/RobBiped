

#ifndef _SIGNALGENERATOR_h
#define _SIGNALGENERATOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

//#include "UpdateServos.h"
#include "../Main/I_PeriodicTask.h"


class SignalGenerator : public I_PeriodicTask{
	
	public:
		
		void init();
		
		void update();
		
		double generateTrajectory();
		
		enum class SignalType { sine, triangular, square, saw };
			
		void configureSignal(SignalType _type, unsigned int _period, unsigned int _amplitude, unsigned int _offset, unsigned int _phaseShift);
		
		
	private:
	
		unsigned long lastCalculatedTime;
		
		SignalType signalType = SignalType::sine;
		
		unsigned int period_ms = 500;
		
		double amplitude = 1;
		
		double offset = 0;
		
		unsigned int phaseShift = 0;
		
		double last_output = 0;
		
	};



#endif

