/*
 * TorsoPosture.h
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

#include "arduino.h"

#include "../../Main/Configs.h"
#include "../../Main/I_PeriodicTask.h"
#include "../../UserInput/Command.h"
#include "../../Utils/ExponentialFilter.h"
#include "../../Utils/Control/PID.h"

namespace Control {

class TorsoPosture : public I_PeriodicTask
{
	private:

	// Serial Commands pointer
	Command* command_;

	PID pid_;

	Configuration::Configs::Control::TorsoPosture *config_;

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

	// Desired torso pitch angle, in radians
	double setpoint_rad_;

	double torso_error_pitch_;
	double last_torso_error_pitch_;
	double diferential_torso_error_pitch_;

	public:

	void assoc_config(Configuration::Configs::Control::TorsoPosture& _config);
	void init();

	/*
	*  @fn void set_setpoint_rad(double& _desired_torso_pitch_angle);
	*  @brief Setter for desired torso pitch angle, in radians.
	*
	*  @param[in] _desired_torso_pitch_angle Desired torso pitch angle, in radians.
	*/
	void set_setpoint_rad(double& _desired_torso_pitch_angle);

	/*
	*  @fn double compute(double& _current_torso_pitch_angle_rad)
	*  @brief Compute controller.
	*  It returns the computed output value, in radians, to be applied to the pitch joints of the hip.
	*
	*  @param[in] _current_torso_pitch_angle_rad Feedback value; Current pitch posture of the torso, in radians.
	*/
	double compute(double& _current_torso_pitch_angle_rad);
};

}	// End namespace Control
