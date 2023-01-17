/*
 * Configs.h
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
		unsigned short squareButton = 15;
		unsigned short thinButton1 = 2;
		unsigned short thinButton2 = 4;
		unsigned short potentiometer1 = 36;
		unsigned short potentiometer2 = 39;
		}userInputPins;
	
	struct ForceSensors {
		struct GPIO {
			uint8_t clock = 27;
			uint8_t dINs[hx711_number] = {33, 32, 35, 34};	// Search upper for hx711_number static const
			}gpio;
		double filter_exp_constant = 0.2;
		uint16_t filter_threshold_value = 500000;
		double calibration_LeftFoot_LeftFrontSensor = 0.091844;
		double calibration_LeftFoot_RightFrontSensor = 0.092108;
		double calibration_LeftFoot_LeftBackSensor = 0.094531;
		double calibration_LeftFoot_RightBackSensor = 0.092348;
		double calibration_RightFoot_LeftFrontSensor = 0.094735;
		double calibration_RightFoot_RightFrontSensor = 0.092529;
		double calibration_RightFoot_LeftBackSensor = 0.093087;
		double calibration_RightFoot_RightBackSensor = 0.094197;
		struct Location {
			int16_t frontBack_separation = 103;
			int16_t leftRight_separation = 37;
			}location_mm;
		}forceSensors;		// Dependent on static const Configuration::hx711_number
	
	struct GyroscpeAccelerometer {
		struct Offsets {
			int ax_o = -35;	// Accelerometer offsets
			int ay_o = -234;
			int az_o = 919;
			int gx_o = 7;		// Gyroscope offsets
			int gy_o = -35;
			int gz_o = -13;
			}offsets;
		}gyroAcc;
};

} // End namespace Configuration

#endif