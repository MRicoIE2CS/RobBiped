/*
 * TrayectoryInterpolator.h
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

#ifndef _TRAJECTORY_INTERPOLATOR_h
#define _TRAJECTORY_INTERPOLATOR_h

#include "arduino.h"

#include "../../Main/I_PeriodicTask.h"

namespace Control {

/*
*  @brief This class offers a means to control a linear trajectory over a single double-typed variable.
*/
class LinearTrajectoryInterpolator : public I_PeriodicTask {

	private:

		// Final target of the trajectory.
		double target_;

		// Transition time for the trajectory.
		uint64_t transition_time_ms_;

		// Initial angle value.
		double initial_value_;

		// Initial milliseconds registered.
		uint64_t initial_millis_;

		// Calculated value of the linear slope.
		double slope_;

		// This flag indicates if the interpolator has been initiated.
		// It is set the first time compute_output() is called.
		// It is reset by calling reset() method.
		bool initiated_ = false;

		// Method that evaluates if a given trajectory calculation has reached the target value.
		bool is_target_reached(double& _calculated_value);

	public:

		/*
		*  @fn void configure_trayectory(const double& _target, const double& _initial_value, const uint64_t& _transition_time_ms)
		*  @brief Configures the desired trajectory.
		*  The initial time of the trajectory is gathered the first time compute_output() method is called.
		*
		*  The configured parameters would be inconsistent if:
		*  - Angle values are equal.
		*  - Transition time is equal to zero.
		*
		*  @param[in] _initial_value Initial angle value.
		*  @param[in] _target Final target of the trajectory.
		*  @param[in] _transition_time_us Transition time, in milliseconds.
		*  @return bool True if successfully configured. False if any configured parameter is inconsistent.
		*/
		bool configure_trayectory(const double& _initial_value, const double& _target, const uint64_t& _transition_time_ms);

		/*
		*  @fn void compute_output(double& _output)
		*  @brief Output computation.
		*  Calling this method without previously having successfully configured the trajectory through
		*  configure_trayectory() method is undefined behavior.
		*
		*  @param[out] _output Output of the controller.
		*  @return bool Indicates when the trajectory is running. Becomes false when the target has been reached.
		*/
		bool compute_output(double& _output);

		/*
		*  @fn void reset()
		*  @brief Resets interpolator.
		*/
		void reset();
};

}	// End namespace Control

#endif
