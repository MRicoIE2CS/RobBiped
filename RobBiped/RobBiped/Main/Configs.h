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

static const uint8_t hx711_number = 2;	// forceSensors configuration
	
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
			uint8_t din_1 = 19;
			uint8_t din_2 = 5;
			}gpio;
		double filter_exp_constant = 0.5;
		}forceSensors;		// Dependent on static const Configuration::hx711_number
};

} // End namespace Configuration

#endif