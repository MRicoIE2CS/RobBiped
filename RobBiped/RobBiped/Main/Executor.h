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

#include "Configs.h"
#include "I_PeriodicTask.h"
#include "../Actuators/JointsManager.h"
#include "../Control/CMTracking/CMTracking.h"
#include "../Control/TorsoPosture/TorsoPosture.h"
#include "../Control/ZMPTracking/Foot_ZMPTracking.h"
#include "../Kinematics_Dynamics/GlobalKinematics.h"
#include "../Sensors/GyroscopeAccelerometerManager.h"
#include "../Sensors/ForceSensorsManager.h"
#include "../UserInput/UserInput.h"
#include "../Utils/Control/TrayectoryInterpolator.h"
#include "../Utils/Control/Waiting.h"
#include "../Utils/Filters/ExponentialFilter.h"
#include "../Utils/SignalGenerator.h"
#include "../Utils/Sources/PregeneratedTrajectory.h"

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
		GlobalKinematics global_kinematics_;
		///// END OBJECT TASKS __//

		/////____________ CONTROLLERS: __//
		Control::CMTracking cm_tracking_controller_;
		Control::Foot_ZMPTracking left_foot_ZMP_tracking_controller_;
		Control::Foot_ZMPTracking right_foot_ZMP_tracking_controller_;
		Control::TorsoPosture torso_posture_controller_;
		///// END CONTROLLERS __//

		/////____________ APPLICATION EXCLUSIVE OBJECTS AND METHODS: __//

		// State machine related objects and methods
		// TODO: Create new class "State" that unites all necessities from a state
		uint8_t state_number = 0;
		void state_machine_switch();
		void state_machine_execution();
		void read_commands();
		void always_executes();
		bool state0_first_time = true;
		bool state0_finished = false;
		void state0_execution();
		bool state1_first_time = true;
		uint8_t state1_phase = 0;
		bool state1_finished = false;
		void state1_execution();

		// State2: Go UP and DOWN routine
		bool state2_first_time = true;
		bool state2_finished = false;
		void state2_execution();

		// State10: Test: X-balance test, with potentiometer manually controlling lateral DSP movement
		// "xbalance", "zmptrx", "torso.on"
		bool state10_first_time = true;
		bool state10_finished = false;
		void state10_execution();

		// State20: Test: Y offline trajectory tracking with X-axis balance
		bool state20_first_time = true;
		bool state20_finished = false;
		uint32_t state20_sin_period = 4000;
		void state20_execution();

		// State30: Test: Y offline trajectory tracking with walking phases change
		// ""
		bool state30_first_time = true;
		bool state30_finished = false;
		double state30_leg_lifting_distance_mm = 40.0;
		uint32_t state30_leg_lifting_time_ms = 500;
		double state30_leg_lifting_target = 0.0;
		bool state30_lifting_finished = false;
		bool state30_lifting_leg = false;
		bool state30_lowering_leg = false;
		bool state30_lowering_leg_finished = false;
		bool state30_swing_leg_started = false;
		bool state30_swing_leg_finished = false;
		Control::LinearTrajectoryInterpolator state30_lifting_leg_interpolator;
		void state30_execution();

		// State40: Test: ZMP tracking
		bool state40_first_time = true;
		bool state40_finished = false;
		bool state40_left_leg_lifted = false;
		bool state40_right_leg_lifted = false;
		uint32_t state40_delay_for_buttons = 1000;
		uint32_t state40_last_millis_for_buttons = millis();
		double state40_desired_hip_height = 280.0;
		double state40_desired_step_width = 100.0;
		double state40_distance_to_lift = 40.0;
		void state40_execution();

		// State50: Test: ZMP tracking
		bool state50_first_time = true;
		bool state50_finished = false;
		bool state50_left_leg_lifted = false;
		bool state50_right_leg_lifted = false;
		uint32_t state50_delay_for_buttons = 1000;
		uint32_t state50_last_millis_for_buttons = millis();
		double state50_desired_hip_height = 280.0;
		double state50_desired_step_width = 100.0;
		double state50_distance_to_lift = 55.0;
		double state50_distance_addition = 40.0;
		void state50_execution();

		// Other state machine flags
		bool application_on = false;
		bool get_up = false;

		ExpFilter some_exp_filter_;
		ExpFilter some_exp_filter_2_;
		
		// Sin periodic signal
		SignalGenerator sin_signal;
		uint32_t sin_period = 4000;

		// Kinematic objects and definitions
		double right_foot_center_ = 0.0;
		GlobalKinematics::WalkingPhase initial_phase_ = GlobalKinematics::WalkingPhase::DSP_left;
		// Defined desired hip height
		double desired_hip_height_ = 260.0;
		// Defined desired step width
		double desired_step_width_ = 150.0;
		// Step distance
		double step_distance_ = 85;
		
		// Control tasks' objects
		double torso_upright_pitch_control_action = 0.0;
		double torso_setpoint_ = 0.00;
		ExpFilter torso_pitch_exp_filter_;

		// Time variable just for printing
		uint32_t debug_millis = millis();

		// Waiting object
		Control::Waiting waiting_;
		// Waiting first time
		bool waiting_first_time_ = true;
		// Waiting time between linear squat movements
		uint64_t waiting_time_ = 2000;

		///// END APPLICATION EXCLUSIVE OBJECTS AND METHODS: __//

		/////____________ PRIVATE METHODS: __//
		void associations();
		void initialize_servo_setpoints();
		void initialize_application();

		void inputs();
		void main_execution();
		void outputs();
		///// END PRIVATE METHODS: __//

	public:

		/////____________ PUBLIC METHODS: __//
		void init();

		void execution();

	};

#endif
