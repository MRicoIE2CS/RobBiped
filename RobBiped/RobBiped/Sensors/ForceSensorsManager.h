
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

// This class manages the use of multiple HX711 ICs in one object, filtering, adjusting and interpreting the obtained values.
// As the use of multiple HX711 and interpretation of the measured magnitudes hardly depends on the HW setup configured for the robot,
// it is needed to modify this class each time the number of hx711 modules or channels is modified.
// It is considered easier modify the class than to program a configurable class for all possible configurations

class ForceSensorsManager : public I_PeriodicTask
{
private:
	
	Multiple_HX711 multiple_hx711;
	
	// Readings
	long Ax64ChannelValue_0;
	long Bx32ChannelValue_0;
	long Ax64ChannelValue_1;
	long Bx32ChannelValue_1;
	long Ax64ChannelValue_2;
	long Bx32ChannelValue_2;
	long Ax64ChannelValue_3;
	long Bx32ChannelValue_3;
	
	// One filter per each measured magnitude
	ExpFilter filter_ch0_Ax64;
	ExpFilter filter_ch0_Bx32;
	ExpFilter filter_ch1_Ax64;
	ExpFilter filter_ch1_Bx32;
	ExpFilter filter_ch2_Ax64;
	ExpFilter filter_ch2_Bx32;
	ExpFilter filter_ch3_Ax64;
	ExpFilter filter_ch3_Bx32;
	
	Configuration::Configs::ForceSensors *config;
	
	void configuration();

public:

	void assocConfig(Configs::ForceSensors &_config);

	void init();

	bool update();
	
	long getValue_LeftFoot_LeftFrontSensor();
	long getValue_LeftFoot_RightFrontSensor();
	long getValue_LeftFoot_LeftBackSensor();
	long getValue_LeftFoot_RightBackSensor();
	long getValue_RightFoot_LeftFrontSensor();
	long getValue_RightFoot_RightFrontSensor();
	long getValue_RightFoot_LeftBackSensor();
	long getValue_RightFoot_RightBackSensor();
	
	unsigned long getLastElapsedTimeBetweenReadings();
	
	void tare_LeftFoot();
	void tare_RightFoot();
};

#endif

