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

#ifdef false

#ifndef _EXECUTOR_h
#define _EXECUTOR_h

#include "Arduino.h"

#include "Configs.h"
#include "I_PeriodicTask.h"
#include "../Actuators/JointsManager.h"
#include "../Control/FootSupport/FootRollCentering.h"
#include "../Control/TorsoPosture/TorsoPosture.h"
#include "../Sensors/GyroscopeAccelerometerManager.h"
#include "../Sensors/ForceSensorsManager.h"
#include "../UserInput/UserInput.h"
#include "../Utils/Control/TrayectoryInterpolator.h"
#include "../Utils/Control/Waiting.h"
#include "../Utils/ExponentialFilter.h"
#include "../Utils/SignalGenerator.h"

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
		///// END OBJECT TASKS __//

		/////____________ APPLICATION EXCLUSIVE OBJECTS AND METHODS: __//

		// State machine related objects and methods
		uint8_t state_number = 0;
		void state_machine_switch();
		void state_machine_execution();
		void read_commands();
		void always_executes();
		bool state0_first_time = true;
		bool state0_finished = false;
		void state0_execution();
		uint8_t state1_phase = 0;
		bool state1_finished = false;
		void state1_execution();
		bool state2_first_time = true;
		bool state2_finished = false;
		void state2_execution();
		uint8_t state3_phase = 0;
		bool state3_finished = false;
		void state3_execution();
		uint8_t state4_phase = 0;
		bool state4_finished = false;
		void state4_execution();
		bool state5_first_time = true;
		bool state5_finished = false;
		void state5_execution();
		uint8_t state6_phase = 0;
		bool state6_finished = false;
		void state6_execution();
		uint8_t state7_phase = 0;
		bool state7_finished = false;
		void state7_execution();

		// Other state machine flags
		bool automatic_force_tare_on = false;

		// AUTOTARE Trajectory generation objects
		// (these are used on other states)
		double autotare_angle_displacement_ = 0.9;
		double autotare_legs_squat_displacement = 20.0;
		double autotare_leg_lift_displacement = 30.0;
		const uint64_t autotare_trajectories_time_ = 3000;
		const double home_position_leg_length_ = 142.0;
		Control::LinearTrajectoryInterpolator autotare_interpolator1_;
		double last_hip_left_angle_compensation = 0.0;
		double last_hip_right_angle_compensation = 0.0;
		
		// Control tasks' objects
		double torso_upright_pitch_control_action = 0.0;
		double torso_setpoint_ = -0.05;
		ExpFilter torso_pitch_exp_filter_;
		double zmp_lateral_deviation_setpoint_ = 0.0;
		ExpFilter left_zmp_lateral_exp_filter_;
		ExpFilter right_zmp_lateral_exp_filter_;

		SignalGenerator squats_unitary_cycle_generator_;
		Control::LinearTrajectoryInterpolator trajectory_interpolator_;
		bool interpolator_running_ = false;
		const double min_desired_leg_length_ = 115.0;
		const double max_desired_leg_length_ = 140.0;
		const double max_desired_squat_angle_ = -0.2;
		double desired_leg_lenght_ = 0.0;
		const uint64_t squats_period_ms_ = 3000;

		// Enabling flag for squats movement
		bool squats_on_ = false;

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

#endif
