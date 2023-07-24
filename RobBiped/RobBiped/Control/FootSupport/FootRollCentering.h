/*
 * FootRollCentering.h
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

#ifndef _FOOT_ROLL_CENTERING_h
#define _FOOT_ROLL_CENTERING_h

#include "arduino.h"

#include "../../Main/Configs.h"
#include "../../Main/I_PeriodicTask.h"
#include "../../UserInput/Command.h"
#include "../../Utils/Control/PID.h"

namespace Control {

class FootRollCentering : public I_PeriodicTask
{
	private:

		// Serial Commands pointer
		Command* command_;

		PID pid_;

		Configuration::Configs::Control::FootRollCentering *config_;

		// PID constants
		double *kp_;
		double *ki_;
		double *kd_;
		// Anti-windup constant
		double *k_windup_;
		// Setpoint weighting constants
		double *proportional_setpoint_weight_;
		double *derivative_setpoint_weight_;
		// Saturation limits, in degrees
		double *lower_saturation_degrees_;
		double *upper_saturation_degrees_;

		// Desired setpoint, in radians
		double setpoint_rad_ = 0.0;

		// ON state of the controller
		bool controller_on = false;

	public:

		void assoc_config(Configuration::Configs::Control::FootRollCentering& _config);
		void init();
		bool is_on();

		/*
		*  @fn void set_setpoint_rad(double& _desired_foot_roll_angle);
		*  @brief Setter for desired foot roll angle, in radians.
		*
		*  @param[in] _desired_foot_roll_angle Desired foot roll angle, in radians.
		*/
		void set_setpoint_rad(double& _desired_zmp_lateral_deviation_mm);

		/*
		*  @fn double compute(double& _current_foot_roll_angle_rad)
		*  @brief Compute controller.
		*  It returns the computed output value, in radians, to be applied to the foot roll joint.
		*
		*  @param[in] _current_foot_roll_angle_rad Feedback value; Current foot roll angle, in radians.
		*/
		double compute(double& _current_foot_zmp_lateral_deviation_mm);
};

}

#endif