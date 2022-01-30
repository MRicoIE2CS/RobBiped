// trialTrajectoryGenerator.h

#ifndef _TRIALTRAJECTORYGENERATOR_h
#define _TRIALTRAJECTORYGENERATOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "UpdateServos.h"


class trialTrajectoryGenerator  : public I_Task{
	public:
		
		void generateTrajectory();
		
		void init();
		
		void update();
	};



#endif

