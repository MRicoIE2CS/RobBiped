/*
 * PID.h
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

#ifndef _PID_h
#define _PID_h

#include "arduino.h"

#include "../Filters/ExponentialFilter.h"
#include "FunctionBocks.h"

namespace Control {

class PID {

	private:

	// TODO: Implement new saturation limits for the integral action

		// PID constants
		double Kp_ = 0.0;
		double Ki_ = 0.0;
		double Kd_ = 0.0;

		// Anti-windup constant
		// If apply_saturation_ = false, no anti-windup can be applied
		// The advice is to chose Kw as a fraction of 1/Ti,
		// with Ti = Kp/Ki.
		double Kw_ = 0.5;

		// Time constant in milliseconds
		uint16_t time_constant_millis_ = 1000;

		// Setpoint weighting for proportional action
		double on_proportional_setpoint_weight_ = 1;
		// Setpoint weighting for derivative action
		double on_derivative_setpoint_weight_ = 0.0;

		// Deadband compensation constants
		double negative_db_compensation = 0.0;
		double positive_db_compensation = 0.0;
		// Deadband compensation
		double apply_inverse_deadband(const double &_sign_variable, const double &_u);

		// Saturation constants
		// By default, no saturation is applied, nor anti-windup technique for integral part
		bool apply_saturation_ = false;
		double lower_limit_;
		double upper_limit_;

		// Memory variables
		uint64_t last_millis_computation = millis();
		double last_setpoint_ = 0.0;
		double last_feedback_ = 0.0;
		double last_proportional_action_ = 0.0;
		double last_integral_action_ = 0.0;
		double last_derivative_action_ = 0.0;
		double last_controller_output_ = 0.0;
		double last_saturated_controller_output_ = 0.0;

		// Differential filter
		ExpFilter diferential_filter_;

		// In sleep mode, control action is not computed, and the internal components do not grow (as the integral part).
		bool sleep_ = true;

	public:

		/*
		*  @fn void set_constants(double& _Kp, double& _Ki, double& _Kd)
		*  @brief Setter for the PID constants.
		*
		*  @param[in] _Kp Proportional constant.
		*  @param[in] _Ki Integral constant.
		*  @param[in] _Kd Derivative constant.
		*/
		void set_constants(double& _Kp, double& _Ki, double& _Kd);

		/*
		*  @fn void set_antiwindup(double& _Kw)
		*  @brief Setter for the anti-windup constant.
		*
		*  @param[in] _Kw Anti-windup constant.
		*/
		void set_antiwindup(double& _Kw);

		/*
		*  @fn void set_time_constant_millis(uint16_t& _millis)
		*  @brief Setter for the time constant, in milliseconds.
		*
		*  @param[in] _millis Time constant, in milliseconds.
		*/
		void set_time_constant_millis(uint16_t& _millis);

		/*
		*  @fn void set_saturation_constants(bool& _apply_saturation = false, double& _lower_limit = 0.0, double& _upper_limit = 0.0)
		*  @brief Setter for the saturation limits to apply on the output of the controller.
		*  This saturation will be used for the anti-windup mechanism of the integral part.
		*  By default (when no calling this method), the PID does not apply saturation limitations to the output of the controller,
		*  nor the anti-windup mechanism.
		*
		*  @param[in] _apply_saturation Enable saturation block and anti-windup mechanism.
		*  @param[in] _lower_limit Lower limit.
		*  @param[in] _upper_limit Upper limit.
		*/
		void set_saturation_constants(bool _apply_saturation, double& _lower_limit, double& _upper_limit);

		/*
		*  @fn void set_setpoint_weighting(double& _on_proportional, double& _on_derivative);
		*  @brief Setter for the setpoint weighting constants.
		*
		*  @param[in] _on_proportional Setpoint weight on proportional.
		*  @param[in] _on_derivative Setpoint weight on derivative.
		*/
		void set_setpoint_weighting(double& _on_proportional, double& _on_derivative);

		/*
		*  @fn void set_deadband_compensation(double& _negative_deadband, double& _positive_deadband)
		*  @brief Setter for the deadband values, for deadband compensation.
		*  If the call is performed without arguments, or with values equal to 0, the deadband compensation is disabled.
		*  (Deadband compensation is disabled by default)
		*
		*  @param[in] _negative_deadband Negative deadband.
		*  @param[in] _positive_deadband Positive deadband.
		*/
		void set_deadband_compensation(const double& _negative_deadband = 0, const double& _positive_deadband = 0);

		/*
		*  @fn void set_derivative_filter_time_constant(const uint32_t& _time_constant_ms)
		*  @brief Sets the time constant of the derivative filter.
		*  The time constant is the amount of time for the smoothed response of a unit step function to reach
		*  1 - 1/e proportion of the original signal, which approximates to 63.2%.
		*
		*  @param[in] _time_constant_ms Time constant in ms.
		*/
		void set_derivative_filter_time_constant(const uint32_t& _time_constant_ms);

		/*
		*  @fn double compute_output(const double& _setpoint, const double& _feedback, const double& _sign_of_deadband = 0)
		*  @fn void compute_output(const double& _setpoint, const double& _feedback, double& _output, const double& _sign_of_deadband = 0)
		*  @brief Output computation.
		*
		*  @param[in] _setpoint Setpoint.
		*  @param[in] _feedback Feedback measurement.
		*  @param[in] _sign_of_deadband Signal that determines the sign of the deadband compensation.
		*  @param[out] _output Output of the controller.
		*/
		double compute_output(const double& _setpoint, const double& _feedback, const double& _sign_of_deadband = 0);
		void compute_output(const double& _setpoint, const double& _feedback, double& _output, const double& _sign_of_deadband = 0);

		/*
		*  @fn void get_control_action_values(const double& _kp, const double& _ki, double& _kd)
		*  @brief Getter for the last values of the separated control components of the PID.
		*
		*  @param[out] _kp Proportional component.
		*  @param[out] _ki Integral component.
		*  @param[out] _kd Derivative component.
		*/
		void get_control_action_values(double& _kp, double& _ki, double& _kd);

		/*
		*  @fn void sleep()
		*  @brief Sets the controller to sleep mode.
		*  In sleep mode, control action is not computed, and the internal components do not grow (as the integral part).
		*  Controller is woken up (sleep state is cleaned) by calling compute_output() method.
		*/
		void sleep();
};

}	// End namespace Control

#endif
