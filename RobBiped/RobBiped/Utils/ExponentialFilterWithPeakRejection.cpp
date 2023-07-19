/*
 * ExponentialFilterWithPeakRejection.cpp
 *
 * Copyright 2023 Mikel Rico Abajo (https://github.com/MRicoIE2CS)

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

void ExpFilterPeakReject::set_threshold_value(double value)
{
	threshold_value_ = value;
}

double ExpFilterPeakReject::filter_pr(double raw_value, bool accept_step)
{
	if (abs(abs(raw_value) - abs(last_filtered_value_) > threshold_value_) && !(accept_step))
	{
		return last_filtered_value_;
	} 
	else
	{
		return filter(raw_value);
	}
}

double ExpFilterPeakReject::filter_pr(uint8_t raw_value, bool accept_step)
{
	if (abs(abs(raw_value) - abs(last_filtered_value_) > threshold_value_) && !(accept_step))
	{
		return last_filtered_value_;
	}
	else
	{
		return filter(raw_value);
	}
}

double ExpFilterPeakReject::filter_pr(int8_t raw_value, bool accept_step)
{
	if (abs(abs(raw_value) - abs(last_filtered_value_) > threshold_value_) && !(accept_step))
	{
		return last_filtered_value_;
	}
	else
	{
		return filter(raw_value);
	}
}

double ExpFilterPeakReject::filter_pr(uint16_t raw_value, bool accept_step)
{
	if (abs(abs(raw_value) - abs(last_filtered_value_) > threshold_value_) && !(accept_step))
	{
		return last_filtered_value_;
	}
	else
	{
		return filter(raw_value);
	}
}

double ExpFilterPeakReject::filter_pr(int16_t raw_value, bool accept_step)
{
	if (abs(abs(raw_value) - abs(last_filtered_value_) > threshold_value_) && !(accept_step))
	{
		return last_filtered_value_;
	}
	else
	{
		return filter(raw_value);
	}
}

double ExpFilterPeakReject::filter_pr(uint32_t raw_value, bool accept_step)
{
	if (abs(abs((int32_t)raw_value) - abs((int32_t)last_filtered_value_) > threshold_value_) && !(accept_step))
	{
		return last_filtered_value_;
	}
	else
	{
		return filter(raw_value);
	}
}

double ExpFilterPeakReject::filter_pr(int32_t raw_value, bool accept_step)
{
	if (abs(abs(raw_value) - abs(last_filtered_value_) > threshold_value_) && !(accept_step))
	{
		return last_filtered_value_;
	}
	else
	{
		return filter(raw_value);
	}
}

double ExpFilterPeakReject::filter_pr(uint64_t raw_value, bool accept_step)
{
	if (abs(abs((int32_t)raw_value) - abs((int32_t)last_filtered_value_) > threshold_value_) && !(accept_step))
	{
		return last_filtered_value_;
	}
	else
	{
		return filter(raw_value);
	}
}

double ExpFilterPeakReject::filter_pr(int64_t raw_value, bool accept_step)
{
	if (abs(abs(raw_value) - abs(last_filtered_value_) > threshold_value_) && !(accept_step))
	{
		return last_filtered_value_;
	}
	else
	{
		return filter(raw_value);
	}
}
