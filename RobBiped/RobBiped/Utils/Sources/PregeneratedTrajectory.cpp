/*
 * PregeneratedTrajectory.cpp
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

#include "PregeneratedTrajectory.h"

void PregeneratedTrajectory::init(){

	if(!LITTLEFS.begin()){
		Serial.println("LITTLEFS Mount Failed");
		return;
	}

	File file = LITTLEFS.open(filename_);
	if(!file){
		Serial.println("Failed to open file for reading");
		return;
	}

	numberof_samples_ = 0;
	while(file.available()){

		String read_str_num = file.readStringUntil(',');

		if (read_str_num.isEmpty())
		{
			continue;
		}

		double number = read_str_num.toDouble();

		trajectory_vector_.push_back(number);
		numberof_samples_++;
	}

	final_time_ms_ = numberof_samples_ * sampling_time_ms_;

	file.close();
}

void PregeneratedTrajectory::start_trajectory()
{
	initial_time_ = millis();
}

void PregeneratedTrajectory::set_file_name(String &_filename)
{
	filename_ = _filename;
}

void PregeneratedTrajectory::set_sampling_time_ms(uint32_t _sampling_time_ms)
{
	sampling_time_ms_ = _sampling_time_ms;
}

double PregeneratedTrajectory::get_value()
{
	uint32_t current_ms = millis();
	uint32_t last_sample = (current_ms - initial_time_) / sampling_time_ms_;
	if (last_sample > numberof_samples_ - 1)
	{
		last_sample = numberof_samples_ - 1;
	}

	double output_value;
	if (last_sample + 1 <= numberof_samples_ - 1)
	{
		uint32_t interpolation_time_ms = current_ms - initial_time_ - sampling_time_ms_ * last_sample;
		double interpolation_time_fraction = (double)interpolation_time_ms / (double)sampling_time_ms_;

		double last_sample_value = trajectory_vector_[last_sample];
		double next_sample_value = trajectory_vector_[last_sample + 1];
		// Interpolated value
		output_value = last_sample_value + ((next_sample_value - last_sample_value) / interpolation_time_fraction);
	}
	else
	{
		// Non interpolated
		output_value = trajectory_vector_[last_sample];
	}

	return output_value;
}
