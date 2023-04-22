/*
 * Waiting.h
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

#ifndef _WAITING_h
#define _WAITING_h

#include "arduino.h"

namespace Control {

/*
*  @brief This class offers a means to control an idle state (as a non-blocking delay) during a configured time.
*/
class Waiting {

	private:

		// Waiting time.
		uint64_t waiting_time_ms_;

		// Initial milliseconds registered.
		uint64_t initial_millis_;

		// Calculated final time
		uint64_t final_millis_;

		// This flag indicates if the waiting process has been initiated.
		// It is set the first time evaluate() is called.
		// It is reset by calling reset() method.
		bool initiated_ = false;

	public:

		/*
		*  @fn void configure_waiting(const uint64_t& _waiting_time_ms)
		*  @brief Configures the desired waiting process.
		*  The initial time of the waiting process is gathered the first time evaluate() method is called.
		*
		*  The configured parameters would be inconsistent if:
		*  - Waiting process time is equal to zero.
		*
		*  @param[in] _transition_time_us Transition time, in milliseconds.
		*  @return bool True if successfully configured. False if any configured parameter is inconsistent.
		*/
		bool configure_waiting(const uint64_t& _waiting_time_ms);

		/*
		*  @fn void evaluate()
		*  @brief Evaluate if the elapsed time is greater than the waiting process time configured.
		*
		*  @return bool Returns true if the elapsed time is greater than the waiting process time configured.
		*/
		bool evaluate();

		/*
		*  @fn void reset()
		*  @brief Resets waiting process.
		*/
		void reset();
};

}	// End namespace Control

#endif
