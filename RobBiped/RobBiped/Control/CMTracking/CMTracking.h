/*
 * CMTracking.h
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


#ifndef _CM_TRACKING_h
#define _CM_TRACKING_h

#include "arduino.h"

#include "../../Kinematics_Dynamics/GlobalKinematics.h"
#include "../../Main/Configs.h"
#include "../../Sensors/ForceSensorsManager.h"
#include "../../UserInput/Command.h"
#include "../../Utils/LinearAlgebra/ArduinoEigenDense.h"
#include "../../Utils/Sources/PregeneratedTrajectory.h"

using Eigen::Vector2d;

namespace Control {

class CMTracking
{
	public:

		enum class Mode { OnlineReference, OfflineReference };

	private:

		// Serial Commands pointer
		Command* command_;

		// Configuration
		Configuration::Configs::Control::CMTracking *config_;
		const double g_ = 9800; // mm/s^2

		// Height of the CM
		double h_;
		
		// Constants for the delayed response on the ZMP tracking
		double Tra_x_, Tra_y_;

		// Proportional constants of the controller
		double d0x, d1x, d2x, d0y, d1y, d2y;

		// Pregenerated trajectories
		PregeneratedTrajectory CM_path_x_;
		PregeneratedTrajectory dCM_path_x_;
		PregeneratedTrajectory ddCM_path_x_;
		PregeneratedTrajectory dddCM_path_x_;
		PregeneratedTrajectory CM_path_y_;
		PregeneratedTrajectory dCM_path_y_;
		PregeneratedTrajectory ddCM_path_y_;
		PregeneratedTrajectory dddCM_path_y_;
		
		// Pointers to signal holders
		GlobalKinematics *global_kinematics_;
		ForceSensorsManager *force_sensor_;

		void start_trajectories();

		void get_reference_signals(Vector2d &_CM_ref, Vector2d &_vCM_ref, Vector2d &_aCM_ref, Vector2d &_jCM_ref);
		void get_feedback_signals(Vector2d &_CM_est, Vector2d &_vCM_est, Vector2d &_aCM_med, Vector2d &_ZMP_med);
		
		// Mode
		Mode mode_x_ = Mode::OfflineReference;
		Mode mode_y_ = Mode::OfflineReference;
		
		// Flag for the running state
		bool is_runnning_ = false;
		
		// Output of the controller
		Vector2d control_action_;

	public:

		// Associations
		void assoc_config(Configuration::Configs::Control::CMTracking &_config);
		void assoc_globalkinematics(GlobalKinematics &global_kinematics_);
		void assoc_sensors(ForceSensorsManager &_force_sensors_manager);
		
		// Initialization
		void init();
		
		// Sets the operating mode
		void set_mode(Mode &_mode_x, Mode &_mode_y);

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
