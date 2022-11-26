/*
 * Configs.h
 *
 * Created: 13/02/2022 20:24:38
 *  Author: MRICO
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
			uint8_t clock = 18;
			uint8_t dINs[hx711_number] = {19, 5, 35, 34};	// Search upper for hx711_number static const
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