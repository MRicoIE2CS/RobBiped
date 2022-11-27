// ExponentialFilterWithPeakRejection.h

#ifndef _EXPONENTIALFILTERWITHPEAKREJECTION_h
#define _EXPONENTIALFILTERWITHPEAKREJECTION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "ExponentialFilter.h"


class ExpFilterPeakReject: public ExpFilter {
	protected:
		double threshold_value;
		
	public:
		void setThresholdValue(double value);
		double filter_pr(double rawValue, bool acceptStep = false);
		double filter_pr(uint8_t rawValue, bool acceptStep = false);
		double filter_pr(int8_t rawValue, bool acceptStep = false);
		double filter_pr(uint16_t rawValue, bool acceptStep = false);
		double filter_pr(int16_t rawValue, bool acceptStep = false);
		double filter_pr(uint32_t rawValue, bool acceptStep = false);
		double filter_pr(int32_t rawValue, bool acceptStep = false);
		double filter_pr(uint64_t rawValue, bool acceptStep = false);
		double filter_pr(int64_t rawValue, bool acceptStep = false);
	};

#endif
