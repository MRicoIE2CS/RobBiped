/*
 * JointsConfig.h
 *
 * Copyright 2023 Mikel Rico Abajo (MRicoIE2C)

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
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, 1.20);
	jointInitializer.calibration_set_min_angle(false, -0.2);
	jointInitializer.calibration_set_max_angle(false, 0.4);
	PCA9685_1_servoMap_[0] = jointInitializer;
// LEFT FOOT - PITCH	
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, 1.28);
	jointInitializer.calibration_set_min_angle(false, -0.6);
	jointInitializer.calibration_set_max_angle(false, 0.8);
	PCA9685_1_servoMap_[1] = jointInitializer;
	
// LEFT KNEE
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, 1.57);
	jointInitializer.calibration_set_min_angle(false, -0.2);
	jointInitializer.calibration_set_max_angle(false, 1.5);
	PCA9685_1_servoMap_[2] = jointInitializer;

// LEFT HIP - PITCH
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, 1.84);
	jointInitializer.calibration_set_min_angle(false, -0.45);
	jointInitializer.calibration_set_max_angle(false, 1);
	PCA9685_1_servoMap_[3] = jointInitializer;
// LEFT HIP - ROLL
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, 1.75);
	jointInitializer.calibration_set_min_angle(false, -0.1);
	jointInitializer.calibration_set_max_angle(false, 0.5);
	PCA9685_1_servoMap_[4] = jointInitializer;
	

// LEFT SHOULDER
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, 1.41);
	jointInitializer.calibration_set_min_angle(false, -0.45);
	jointInitializer.calibration_set_max_angle(false, 0.45);
	PCA9685_1_servoMap_[5] = jointInitializer;
// LEFT SHOULDER
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, 1.37);
	jointInitializer.calibration_set_min_angle(false, -0.45);
	jointInitializer.calibration_set_max_angle(false, 0.45);
	PCA9685_1_servoMap_[6] = jointInitializer;
	
// UNUSED
	jointInitializer.clean_calibration_values();
	PCA9685_1_servoMap_[7] = jointInitializer;
	PCA9685_1_servoMap_[8] = jointInitializer;

// RIGHT SHOULDER
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, 1.67);
	jointInitializer.calibration_set_min_angle(false, -0.45);
	jointInitializer.calibration_set_max_angle(false, 0.45);
	PCA9685_1_servoMap_[9] = jointInitializer;
// RIGHT SHOULDER
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, 1.67);
	jointInitializer.calibration_set_min_angle(false, -0.45);
	jointInitializer.calibration_set_max_angle(false, 0.45);
	PCA9685_1_servoMap_[10] = jointInitializer;

// RIGHT HIP - ROLL
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, 1.46);
	jointInitializer.calibration_set_min_angle(false, -0.45);
	jointInitializer.calibration_set_max_angle(false, 0.45);
	PCA9685_1_servoMap_[11] = jointInitializer;
// RIGHT HIP - PITCH
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, 1.64);
	jointInitializer.calibration_set_min_angle(false, -0.45);
	jointInitializer.calibration_set_max_angle(false, 0.45);
	PCA9685_1_servoMap_[12] = jointInitializer;
	
// RIGHT KNEE
	jointInitializer.invert_angle_sign(false);
	jointInitializer.calibration_set_zero(false, 1.87);
	jointInitializer.calibration_set_min_angle(false, -1.5);
	jointInitializer.calibration_set_max_angle(false, 0.2);
	PCA9685_1_servoMap_[13] = jointInitializer;
	
//RIGHT FOOT - ROLL
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, 1.38);
	jointInitializer.calibration_set_min_angle(false, -0.8);
	jointInitializer.calibration_set_max_angle(false, 0.6);
	PCA9685_1_servoMap_[14] = jointInitializer;
//RIGHT FOOT - PITCH
	jointInitializer.invert_angle_sign(true);
	jointInitializer.calibration_set_zero(false, 1.67);
	jointInitializer.calibration_set_min_angle(false, -0.4);
	jointInitializer.calibration_set_max_angle(false, 0.2);
	PCA9685_1_servoMap_[15] = jointInitializer;
}