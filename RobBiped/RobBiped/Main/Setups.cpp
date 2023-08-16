/*
 * Setups.cpp
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

#include "Executor.h"

void Executor::associations()
{
	user_input_.assoc_GPIO(config_.user_input_pins);
	force_sensors_manager_.assoc_config(config_.force_sensors);
	gyroscope_accelerometer_manager_.assoc_config(config_.gyro_acc);
	torso_posture_controller_.assoc_config(config_.control.torso_posture);
// 	left_foot_roll_centering_controller_.assoc_config(config_.control.foot_roll_centering);
// 	right_foot_roll_centering_controller_.assoc_config(config_.control.foot_roll_centering);
	global_kinematics_.assoc_config(config_.kinematics);
	global_kinematics_.assoc_sensors(force_sensors_manager_, gyroscope_accelerometer_manager_);
}

void Executor::initialize_servo_setpoints()
{
	double initial_setpoint = 0.0;
	// Servo setpoint assignation
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootRoll, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipRoll, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftShoulderSagittal, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftShoulderAmplitude, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::Unused1, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::Unused2, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightShoulderAmplitude, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightShoulderSagittal, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipRoll, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, initial_setpoint);
	servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootRoll, initial_setpoint);
}
