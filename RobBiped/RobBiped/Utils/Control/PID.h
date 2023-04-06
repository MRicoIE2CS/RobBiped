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

#include "FunctionBocks.h"

namespace Control {

class PID {

	private:

		// PID constants
		double Kp_ = 0.0;
		double Ki_ = 0.0;
		double Kd_ = 0.0;

		// Time constant in milliseconds
		uint16_t time_constant_millis_ = 1000;

		// Setpoint weighting for proportional action
		double on_proportional_setpoint_weight_ = 1;
		// Setpoint weighting for derivative action
		double on_derivative_setpoint_weight_ = 0;

		// Saturation constants
		// By default, no saturation is applied, nor anti-windup technique for integral part
		bool apply_saturation_ = false;
		double lower_limit_;
		double upper_limit_;

		// Memory variables
		double last_setpoint_ = 0.0;
		double last_feedback_ = 0.0;

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
		*  @fn void set_time_constant_millis(uint16_t& _millis)
		*  @brief Setter for the time constant, in milliseconds.
		*
		*  @param[in] _millis Time constant, in milliseconds.
		*/
		void set_time_constant_millis(uint16_t& _millis);

		/*
		*  @fn void set_saturation_constants(bool& _apply_saturation = false, double& _lower_limit = 0.0, double& _upper_limit = 0.0);
		*  @brief Setter for the saturation limits to apply on the output of the controller.
		*  This saturation will be used for the anti-windup mechanism of the integral part.
		*  By default (when no calling this method), the PID does not apply saturation limitations to the output of the controller,
		*  nor the anti-windup mechanism.
		*
		*  @param[in] _apply_saturation Enable saturation block and anti-windup mechanism.
		*  @param[in] _lower_limit Lower limit.
		*  @param[in] _upper_limit Upper limit.
		*/
		void set_saturation_constants(bool& _apply_saturation, double& _lower_limit, double& _upper_limit);

		/*
		*  @fn void set_setpoint_weighting(double& _on_proportional, double& _on_derivative);
		*  @brief Setter for the setpoint weighting constants.
		*
		*  @param[in] _on_proportional Setpoint weight on proportional.
		*  @param[in] _on_derivative Setpoint weight on derivative.
		*/
		void set_setpoint_weighting(double& _on_proportional, double& _on_derivative);

		/*
		*  @fn void compute_output(const double& _setpoint, const double& _feedback, double& _output);
		*  @brief Output computation.
		*
		*  @param[in] _setpoint Setpoint.
		*  @param[in] _feedback Feedback measurement.
		*  @param[out] _output Output of the controller.
		*/
		void compute_output(const double& _setpoint, const double& _feedback, double& _output);
};

}	// End namespace Control

#endif
