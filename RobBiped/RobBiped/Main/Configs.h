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

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

namespace Configuration
{

static const uint8_t hx711_number = 4;	// forceSensors configuration
	
struct Configs 
{
	struct UserInputPins {
		uint8_t forward_button = 2;
		uint8_t back_button = 4;
		uint8_t potentiometer1 = 36;
		uint8_t potentiometer2 = 39;
		}user_input_pins;
	
	struct ForceSensors {
		struct GPIO {
			uint8_t clock = 27;
			uint8_t dINs[hx711_number] = {33, 32, 35, 34};	// Search upper for hx711_number static const
			}gpio;
		double filter_exp_constant = 0.2;
		uint16_t filter_threshold_value = 500000;
		double calibration_LeftFoot_LeftFront_cell = 0.091844;
		double calibration_LeftFoot_RightFront_cell = 0.092108;
		double calibration_LeftFoot_LeftBack_cell = 0.094531;
		double calibration_LeftFoot_RightBack_cell = 0.092348;
		double calibration_RightFoot_LeftFront_cell = 0.094735;
		double calibration_RightFoot_RightFront_cell = 0.092529;
		double calibration_RightFoot_LeftBack_cell = 0.093087;
		double calibration_RightFoot_RightBack_cell = 0.094197;
		struct Location {
			int16_t frontBack_separation = 103;
			int16_t leftRight_separation = 37;
			}location_mm;
		}force_sensors;		// Dependent on static const Configuration::hx711_number
	
	struct GyroscpeAccelerometer {
		struct Offsets {
			int16_t ax_o = -35;	// Accelerometer offsets
			int16_t ay_o = -234;
			int16_t az_o = 919;
			int16_t gx_o = 7;		// Gyroscope offsets
			int16_t gy_o = -35;
			int16_t gz_o = -13;
			}offsets;
		}gyro_acc;
};

} // End namespace Configuration

#endif