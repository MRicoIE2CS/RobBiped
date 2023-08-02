/*
 * JointsConfig.h
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

#include "JointsManager.h"

void JointsManager::joints_config(){

	Joint jointInitializer;

	//jointInitializer.cleanCalibrationValues();

// LEFT FOOT - ROLL
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, 0.3);
	jointInitializer.calibration_set_min_angle(false, -0.6);
	jointInitializer.calibration_set_max_angle(false, 0.6);
	PCA9685_1_servo_map_[0] = jointInitializer;
// LEFT FOOT - PITCH	
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, -0.3);
	jointInitializer.calibration_set_min_angle(false, -1.0);
	jointInitializer.calibration_set_max_angle(false, 0.8);
	PCA9685_1_servo_map_[1] = jointInitializer;

// LEFT KNEE
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, 0.0);
	jointInitializer.calibration_set_min_angle(false, -1.57);	// This limit is currently forced by servo assembly!
	jointInitializer.calibration_set_max_angle(false, 0.25);
	PCA9685_1_servo_map_[2] = jointInitializer;

// LEFT HIP - PITCH
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, 0.4);
	jointInitializer.calibration_set_min_angle(false, -1);
	jointInitializer.calibration_set_max_angle(false, 1.2);
	PCA9685_1_servo_map_[3] = jointInitializer;
// LEFT HIP - ROLL
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, 0.1);
	jointInitializer.calibration_set_min_angle(false, -0.2);
	jointInitializer.calibration_set_max_angle(false, 0.75);
	PCA9685_1_servo_map_[4] = jointInitializer;

// LEFT SHOULDER FORWARD
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, 0.15);
	jointInitializer.calibration_set_min_angle(false, -0.45);
	jointInitializer.calibration_set_max_angle(false, 0.45);
	PCA9685_1_servo_map_[5] = jointInitializer;
// LEFT SHOULDER LATERAL
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, -0.2);
	jointInitializer.calibration_set_min_angle(false, -0.45);
	jointInitializer.calibration_set_max_angle(false, 0.45);
	PCA9685_1_servo_map_[6] = jointInitializer;

// UNUSED
	jointInitializer.clean_calibration_values();
	PCA9685_1_servo_map_[7] = jointInitializer;
	PCA9685_1_servo_map_[8] = jointInitializer;

// RIGHT SHOULDER LATERAL
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, -0.1);
	jointInitializer.calibration_set_min_angle(false, -0.45);
	jointInitializer.calibration_set_max_angle(false, 0.45);
	PCA9685_1_servo_map_[9] = jointInitializer;
// RIGHT SHOULDER FORWARD
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, 0.1);
	jointInitializer.calibration_set_min_angle(false, -0.45);
	jointInitializer.calibration_set_max_angle(false, 0.45);
	PCA9685_1_servo_map_[10] = jointInitializer;

// RIGHT HIP - ROLL
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, -0.1);
	jointInitializer.calibration_set_min_angle(false, -0.75);
	jointInitializer.calibration_set_max_angle(false, 0.2);
	PCA9685_1_servo_map_[11] = jointInitializer;
// RIGHT HIP - PITCH
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, 0.05);
	jointInitializer.calibration_set_min_angle(false, -1);
	jointInitializer.calibration_set_max_angle(false, 1.2);
	PCA9685_1_servo_map_[12] = jointInitializer;

// RIGHT KNEE
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, 0.3);
	jointInitializer.calibration_set_min_angle(false, -1.57);	// This limit is currently forced by servo assembly!
	jointInitializer.calibration_set_max_angle(false, 0.25);
	PCA9685_1_servo_map_[13] = jointInitializer;

//RIGHT FOOT - PITCH
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, 0.2);
	jointInitializer.calibration_set_min_angle(false, -1);
	jointInitializer.calibration_set_max_angle(false, 0.8);
	PCA9685_1_servo_map_[14] = jointInitializer;
//RIGHT FOOT - ROLL
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, -0.1);
	jointInitializer.calibration_set_min_angle(false, -0.6);
	jointInitializer.calibration_set_max_angle(false, 0.6);
	PCA9685_1_servo_map_[15] = jointInitializer;

	uint8_t aux_idx = 0;
	for (auto joint : PCA9685_1_servo_map_)
	{
		last_joint_setpoints_[static_cast<Configuration::JointsNames>(aux_idx)] = 0.0;
		aux_idx++;
	}
}
