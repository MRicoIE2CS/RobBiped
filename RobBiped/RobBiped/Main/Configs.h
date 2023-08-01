/*
 * Configs.h
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

#ifndef _CONFIGS_h
#define _CONFIGS_h

#include "Arduino.h"

namespace Configuration
{

enum class JointsNames {LeftFootRoll = 0, LeftFootPitch = 1, LeftKnee = 2, LeftHipPitch = 3,
						LeftHipRoll = 4, LeftShoulderSagittal = 5, LeftShoulderAmplitude = 6,
						Unused1 = 7, Unused2 = 8,
						RightShoulderAmplitude = 9, RightShoulderSagittal = 10, RightHipRoll = 11,
						RightHipPitch = 12, RightKnee = 13, RightFootPitch = 14, RightFootRoll = 15 };

static const uint8_t hx711_number = 8;	// forceSensors configuration

struct Configs 
{
	struct UserInputPins {
		uint8_t forward_button = 2;
		uint8_t back_button = 4;
		uint8_t potentiometer1 = 36;
		uint8_t potentiometer2 = 39;
		}user_input_pins;

	struct ForceSensors {
		// Changes in the configuration of the load cells and the channels used might need modifications in ForceSensorsManager class.
		struct GPIO {
			uint8_t clock = 27;
			uint8_t dINs[hx711_number] = {33, 16, 32, 17, 34, 19, 35, 18};	// Order is important! Search upper for hx711_number static const
			}gpio;
		struct ActiveChannels {
			bool _Ax128 = false;
			bool _Ax64 = true;
			bool _Bx32 = false;
			}active_channels;
		double filter_time_constant_ms = 40;
		uint16_t filter_threshold_value = 500000;
		double calibration_LeftFoot_LeftFront_cell = 0.048;	// Grams per unit
		double calibration_LeftFoot_RightFront_cell = 0.048;
		double calibration_LeftFoot_LeftBack_cell = 0.048;
		double calibration_LeftFoot_RightBack_cell = 0.048;
		double calibration_RightFoot_LeftFront_cell = 0.048;
		double calibration_RightFoot_RightFront_cell = 0.048;
		double calibration_RightFoot_LeftBack_cell = 0.048;
		double calibration_RightFoot_RightBack_cell = 0.048;
		struct Location {
			int16_t frontBack_separation = 103;
			int16_t leftRight_separation = 37;
			}location_mm;
		uint32_t touch_detection_up_threshold_gr = 175;
		uint32_t touch_detection_down_threshold_gr = 125;
		}force_sensors;		// Dependent on static const Configuration::hx711_number

	struct GyroscpeAccelerometer {
		struct Offsets {
			int16_t ax_o = -303;	// Accelerometer offsets
			int16_t ay_o = -214;
			int16_t az_o = 911;
			int16_t gx_o = 1;		// Gyroscope offsets
			int16_t gy_o = -35;
			int16_t gz_o = -15;
			}offsets;
		}gyro_acc;

	struct Kinematics {
		double d_hip_width = 90.4;		// Lateral distance between both hip roll joints (lh)
		double height_hip = 85;			// Height between joints of the hip (l1)
		double height_hip_knee = 79;	// Height between knee and the first joint of the hip (thigh) (l2)
		double height_knee_ankle = 63;	// Height between ankle and knee (calf) (l3)
		double height_ankle = 45;		// Height between joints of the ankle (l4)
		double height_foot = 35;		// Height between floor and first joint of the ankle (l5)
		double d_lateral_foot = 13;		// Lateral distance between foot center and the axis of the first ankle joint (a1)
		}kinematics;


	struct PCA9685 {
		
		}pca9685;

	struct Control {

		struct TorsoPosture {
			// PID constants
			double kp = 0.5;
			double ki = 0.1;
			double kd = 0.0;
			// Anti-windup constant
			double k_windup = 0.5;
			// Setpoint weighting constants
			double proportional_setpoint_weight = 1.0;
			double derivative_setpoint_weight = 0.0;
			// Saturation limits
			double lower_saturation_degrees = -45;
			double upper_saturation_degrees = 45;
			}torso_posture;

		struct FootRollCentering {
				// PID constants
				double kp = 0.001;
				double ki = 0.001;
				double kd = 0.0;
				// Anti-windup constant
				double k_windup = 5;
				// Setpoint weighting constants
				double proportional_setpoint_weight = 1.0;
				double derivative_setpoint_weight = 0.0;
				// Saturation limits
				double lower_saturation_degrees = -40;
				double upper_saturation_degrees = 40;
			}foot_roll_centering;

		}control;
};

} // End namespace Configuration

#endif
