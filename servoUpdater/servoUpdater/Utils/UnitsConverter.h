// UnitsConverter.h

#ifndef _UNITSCONVERTER_h
#define _UNITSCONVERTER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

struct UnitsConvert {
	
	struct Angle {
		
		double degToRad(double _ang);
		double radToDeg(double _ang);
		
		};
	
	
	};


#endif

