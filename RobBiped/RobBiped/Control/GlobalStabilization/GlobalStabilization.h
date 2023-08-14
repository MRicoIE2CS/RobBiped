/*
 * GlobalStabilization.h
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


#ifndef _GLOBAL_STABILIZATION_h
#define _GLOBAL_STABILIZATION_h

#include "arduino.h"

#include "../../Kinematics_Dynamics/GlobalKinematics.h"
#include "../../Main/Configs.h"
#include "../../Sensors/GyroscopeAccelerometerManager.h"
#include "../../Sensors/ForceSensorsManager.h"
#include "../../UserInput/Command.h"
#include "../../Utils/LinearAlgebra/ArduinoEigenDense.h"
#include "../../Utils/Sources/PregeneratedTrajectory.h"

using Eigen::Vector2d;

namespace Control {

class GlobalStabilization
{
	public:

		enum class Mode { Stabilization, Tracking };

	private:

		// Serial Commands pointer
		Command* command_;

		// Configuration
		Configuration::Configs::Control::CMTracking *config_;
		const double g_ = 9800; // mm/s^2

		// Height of the CM
		double h_;

		// Pregenerated trajectories
		PregeneratedTrajectory CM_path_y_;
		String CM_path_y_filename_ = "/CM_y.txt";
		uint32_t CM_path_y_sampletime_ = 10;
		PregeneratedTrajectory dCM_path_y_;
		String dCM_path_y_filename_ = "/dCM_y.txt";
		uint32_t dCM_path_y_sampletime_ = 10;
		PregeneratedTrajectory ddCM_path_y_;
		String ddCM_path_y_filename_ = "/ddCM_y.txt";
		uint32_t ddCM_path_y_sampletime_ = 10;
		PregeneratedTrajectory dddCM_path_y_;
		String dddCM_path_y_filename_ = "/dddCM_y.txt";
		uint32_t dddCM_path_y_sampletime_ = 10;
		
		// Pointers to signal holders
		GlobalKinematics *global_kinematics_;
		GyroscopeAccelerometerManager *gyroacc_sensor_;
		ForceSensorsManager *force_sensor_;

		void get_all_signals(double _reference_signals_x[4], double _reference_signals_y[4], double _feedback_signals_x[4], double _feedback_signals_y[4]);
		
		// Mode
		Mode mode_ = Mode::Tracking;
		
		// Flag for the running state
		bool is_runnning_ = false;
		
		// Output of the controller
		Vector2d control_action_;

	public:

		// Associations
		void assoc_config(Configuration::Configs::Control::CMTracking &_config);
		void assoc_globalkinematics(GlobalKinematics &global_kinematics_);
		void assoc_sensors(ForceSensorsManager &_force_sensors_manager, GyroscopeAccelerometerManager &_gyroscope_accelerometer_manager);
		
		// Sets the operating mode
		void set_mode(Mode &_mode);

		// Reset the pregenerated trajectory tracking
		// Sets the running state of the controller to off
		void reset_trajectory();

		// Computes and returns the ZMP action
		Vector2d compute_ZMP_action();
		// Returns the computed ZMP action
		Vector2d get_ZMP_action();
};

}	// End namespace Control

#endif
