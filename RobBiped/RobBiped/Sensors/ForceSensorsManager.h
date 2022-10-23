
/*
 * ForceSensorsManager.h
 *
 * Created: 20/10/2022
 * Author: MRICO
 */ 

#ifndef _FORCESENSORSMANAGER_h
#define _FORCESENSORSMANAGER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "../Main/I_PeriodicTask.h"
#include "../Main/Configs.h"
#include "HX711/multiple_HX711.h"
#include "../Utils/ExponentialFilter.h"

using namespace Configuration;

// TODO: Fine calibration mechanism
// TODO: Units translation
// TODO: Tare left foot and tare right foot methods
// TODO: getValue methods (Assign location for every load cell

class ForceSensorsManager : public I_PeriodicTask
{
private:
	
	Multiple_HX711 multiple_hx711;
	ExpFilter filter_ch0_Ax64;
	ExpFilter filter_ch0_Bx32;
	ExpFilter filter_ch1_Ax64;
	ExpFilter filter_ch1_Bx32;
	
	long Ax64ChannelValue_0;
	long Bx32ChannelValue_0;
	long Ax64ChannelValue_1;
	long Bx32ChannelValue_1;
	
	Configuration::Configs::ForceSensors *config;
	
	void configuration();

public:

	void assocConfig(Configs::ForceSensors &_config);

	void init();

	void update();
	
	long getValue_LeftFoot_LeftFrontSensor();
	long getValue_LeftFoot_RightFrontSensor();
	long getValue_LeftFoot_LeftBackSensor();
	long getValue_LeftFoot_RightBackSensor();
	long getValue_RightFoot_LeftFrontSensor();
	long getValue_RightFoot_RightFrontSensor();
	long getValue_RightFoot_LeftBackSensor();
	long getValue_RightFoot_RightBackSensor();
};

#endif

