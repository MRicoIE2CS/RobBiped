/*
 * WalkingPhasesManager.h
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

#ifndef _WALKING_PHASES_MANAGER_h
#define _WALKING_PHASES_MANAGER_h

#include "Arduino.h"

// TODO: Perform here all the logic related to Walking Phases management

class WalkingPhasesManager {
	public:
		// Walking phases:
		// DSP_left -> DSP and leaning weight towards the left foot (moving to the left)
		// DSP_right -> DSP and leaning weight towards the right foot (moving to the right)
		// SSP_left -> SSP with weight on the left foot
		// SSP_right -> SSP with weight on the right foot
		enum class WalkingPhase {DSP_left = 0, DSP_right = 1, SSP_left = 2, SSP_right = 3 };

		bool check_walking_phase();
		WalkingPhase get_walking_phase();

		// TODO

	private:
		WalkingPhase phase_;

		// TODO
	};

#endif
