/*
 * PregeneratedTrajectory.h
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

#ifndef _PREGENERATED_TRAJECTORY_h
#define _PREGENERATED_TRAJECTORY_h

#include "Arduino.h"

#include <vector>

#include "../../Main/I_PeriodicTask.h"
#include "../FilesystemStorage/LittleFS/LITTLEFS.h"

using fs::LITTLEFSFS;

// This class reads an specified file within the LittleFS filesystem,
// and parses the file, getting interpolated samples of the signal.
// The file must be formatted as a comma separated values (CSV) file.
class PregeneratedTrajectory : public I_PeriodicTask{

	private:

	uint32_t initial_time_;
	
	uint32_t sampling_time_ms_ = 10;

	String filename_ = "/pregenerated_trajectory.txt";

	std::vector<double> trajectory_vector_;
	uint32_t numberof_samples_;
	uint32_t final_time_ms_;

	public:

	void set_file_name(String &_filename);

	void set_sampling_time_ms(uint32_t _sampling_time_ms);

	void init();

	void start_trajectory();

	double get_value();
};

#endif
