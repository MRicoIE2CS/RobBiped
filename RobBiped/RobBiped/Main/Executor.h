/*
 * Executor.h
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

#ifndef _EXECUTOR_h
#define _EXECUTOR_h

#include "Arduino.h"

#include "I_PeriodicTask.h"
#include "../Actuators/JointsManager.h"
#include "../Utils/SignalGenerator.h"
#include "Configs.h"
#include "../Utils/ExponentialFilter.h"
#include "../UserInput/UserInput.h"
#include "../Sensors/ForceSensorsManager.h"
#include "../Sensors/GyroscopeAccelerometerManager.h"
#include "../Control/TorsoPosture/TorsoPosture.h"
#include "../Control/FootSupport/FootRollCentering.h"

class Executor {

	private:

		/////____________ OBJECTS: __//
		Configuration::Configs config_;
		Command* command_;	// Serial Commands pointer
		///// END OBJECTS: __//

		/////____________ TASK OBJECTS: __//
		UserInput user_input_;
		JointsManager servo_updater_;
		ForceSensorsManager force_sensors_manager_;
		GyroscopeAccelerometerManager gyroscope_accelerometer_manager_;
		Control::TorsoPosture torso_posture_controller_;
		Control::FootRollCentering left_foot_roll_centering_controller_;
		Control::FootRollCentering right_foot_roll_centering_controller_;

		SignalGenerator squats_unitary_cycle_generator_;
		///// END OBJECT TASKS __//

		/////____________ AUXILIARY OBJECTS: __//
		uint8_t state_number = 0;

		double torso_setpoint_ = 0.0;
		ExpFilter torso_pitch_exp_filter_;
		double zmp_lateral_deviation_setpoint_ = 0.0;
		ExpFilter left_zmp_lateral_exp_filter_;
		ExpFilter right_zmp_lateral_exp_filter_;

		bool squats_on = false;
		bool squats_first_time = true;
		///// END AUXILIARY OBJECTS: __//

		/////____________ PRIVATE METHODS: __//
		void associations();
		void initialize_servo_setpoints();

		void inputs();
		void main_execution();
		void outputs();

		// State machine related methods
		void state_machine_switch();
		void state_machine_execution();
		void read_commands();
		void state0_execution();
		void state1_execution();
		void state2_execution();
		///// END PRIVATE METHODS: __//

	public:

		/////____________ PUBLIC METHODS: __//
		void init();

		void execution();

	};

#endif
