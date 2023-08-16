/*
 * Foot_ZMPTracking.h
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

#ifndef _FOOT_ZMP_TRACKING_h
#define _FOOT_ZMP_TRACKING_h

#include "arduino.h"

#include "../../Main/Configs.h"
#include "../../Main/I_PeriodicTask.h"
#include "../../UserInput/Command.h"
#include "../../Utils/Control/PID.h"
#include "../../Utils/LinearAlgebra/ArduinoEigenDense.h"

using Eigen::Vector2d;

namespace Control {

// Controller that gets a desired ZMP setpoint, for X and Y axis, and the local ZMP position feedback,
// then returns a pitch (X) or roll(Y) angle setpoints.
class Foot_ZMPTracking : public I_PeriodicTask
{
	private:

		// Serial Commands pointer
		Command* command_;

		PID pid_x_;
		PID pid_y_;

		Configuration::Configs::Control::Foot_ZMPTracking_x *config_x_;
		Configuration::Configs::Control::Foot_ZMPTracking_y *config_y_;

		Configuration::Configs::Control::Foot_ZMPTracking_x::PID *conf_pid_x_;
		Configuration::Configs::Control::Foot_ZMPTracking_y::PID *conf_pid_y_;

		Configuration::Configs::Control::Foot_ZMPTracking_x::FeedforwardCurve *conf_curve_x_;
		Configuration::Configs::Control::Foot_ZMPTracking_y::FeedforwardCurve *conf_curve_y_;

		Configuration::Configs::Control::Foot_ZMPTracking_x::DeadbandCompensation *conf_db_x_;
		Configuration::Configs::Control::Foot_ZMPTracking_y::DeadbandCompensation *conf_db_y_;

		// Desired setpoint, in mm
		double setpoint_x_mm_ = 0.0;
		double setpoint_y_mm_ = 0.0;

		// Output action, in rad
		Vector2d output_rad;

		// ON state of the controller
		bool controller_x_on = false;
		bool controller_y_on = false;

		// Read serial commands
		void read_commands();

		double custom_curve_x(Configuration::Configs::Control::Foot_ZMPTracking_x::FeedforwardCurve &_curve);

	public:

		void assoc_config(Configuration::Configs::Control::Foot_ZMPTracking_x& _config);
		void assoc_config(Configuration::Configs::Control::Foot_ZMPTracking_y& _config);
		void configure();

		/*
		*  @fn void set_setpoint_x_mm(double& _desired_zmp_lateral_deviation_mm)
		*  @fn void set_setpoint_y_mm(double& _desired_zmp_lateral_deviation_mm)
		*  @brief Setter for desired ZMP setpoint, in mm.
		*
		*  @param[in] _desired_zmp_lateral_deviation_mm Desired ZMP setpoint, in mm.
		*/
		void set_setpoint_x_mm(double& _desired_zmp_lateral_deviation_mm);
		void set_setpoint_y_mm(double& _desired_zmp_lateral_deviation_mm);

		/*
		*  @fn Vector2d compute(double& _x_zmp_feedback, double& _y_zmp_feedback)
		*  @brief Compute controller.
		*  It returns the computed output value, in radians, to be applied to the foot pitch/roll joint.
		*
		*  @param[in] _current_foot_zmp_lateral_deviation_mm Feedback value; Current ZMP deviation, in mm.
		*/
		Vector2d compute(double& _x_zmp_feedback, double& _y_zmp_feedback);
		double compute_x(double& _x_zmp_feedback);
		double compute_y(double& _y_zmp_feedback);

		Vector2d get_control_action();

		bool switch_x_on();
		bool switch_y_on();
		bool switch_x_off();
		bool switch_y_off();

		bool is_x_on();
		bool is_y_on();
};

}

#endif