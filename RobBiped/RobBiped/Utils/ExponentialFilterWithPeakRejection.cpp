// 
// 
// 

#include "ExponentialFilterWithPeakRejection.h"

void ExpFilterPeakReject::setThresholdValue(double value)
{
	threshold_value = value;
}

double ExpFilterPeakReject::filter_pr(double rawValue, bool acceptStep)
{
	if (abs(abs(rawValue) - abs(lastFilteredValue) > threshold_value) && !(acceptStep))
	{
		return lastFilteredValue;
	} 
	else
	{
		return filter(rawValue);
	}
}

double ExpFilterPeakReject::filter_pr(uint8_t rawValue, bool acceptStep)
{
	if (abs(abs(rawValue) - abs(lastFilteredValue) > threshold_value) && !(acceptStep))
	{
		return lastFilteredValue;
	}
	else
	{
		return filter(rawValue);
	}
}

double ExpFilterPeakReject::filter_pr(int8_t rawValue, bool acceptStep)
{
	if (abs(abs(rawValue) - abs(lastFilteredValue) > threshold_value) && !(acceptStep))
	{
		return lastFilteredValue;
	}
	else
	{
		return filter(rawValue);
	}
}

double ExpFilterPeakReject::filter_pr(uint16_t rawValue, bool acceptStep)
{
	if (abs(abs(rawValue) - abs(lastFilteredValue) > threshold_value) && !(acceptStep))
	{
		return lastFilteredValue;
	}
	else
	{
		return filter(rawValue);
	}
}

double ExpFilterPeakReject::filter_pr(int16_t rawValue, bool acceptStep)
{
	if (abs(abs(rawValue) - abs(lastFilteredValue) > threshold_value) && !(acceptStep))
	{
		return lastFilteredValue;
	}
	else
	{
		return filter(rawValue);
	}
}

double ExpFilterPeakReject::filter_pr(uint32_t rawValue, bool acceptStep)
{
	if (abs(abs(rawValue) - abs(lastFilteredValue) > threshold_value) && !(acceptStep))
	{
		return lastFilteredValue;
	}
	else
	{
		return filter(rawValue);
	}
}

double ExpFilterPeakReject::filter_pr(int32_t rawValue, bool acceptStep)
{
	if (abs(abs(rawValue) - abs(lastFilteredValue) > threshold_value) && !(acceptStep))
	{
		return lastFilteredValue;
	}
	else
	{
		return filter(rawValue);
	}
}

double ExpFilterPeakReject::filter_pr(uint64_t rawValue, bool acceptStep)
{
	if (abs(abs(rawValue) - abs(lastFilteredValue) > threshold_value) && !(acceptStep))
	{
		return lastFilteredValue;
	}
	else
	{
		return filter(rawValue);
	}
}

double ExpFilterPeakReject::filter_pr(int64_t rawValue, bool acceptStep)
{
	if (abs(abs(rawValue) - abs(lastFilteredValue) > threshold_value) && !(acceptStep))
	{
		return lastFilteredValue;
	}
	else
	{
		return filter(rawValue);
	}
}
