/*
 * ExponentialFilterWithPeakRejection.cpp
 *
 * Copyright 2023 Mikel Rico Abajo (MRicoIE2C)

 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */ 

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
