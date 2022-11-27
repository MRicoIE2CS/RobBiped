
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
#include "../Utils/ExponentialFilterWithPeakRejection.h"

using namespace Configuration;

// This class manages the use of multiple HX711 ICs in one object, filtering, adjusting and interpreting the obtained values.
// As the use of multiple HX711 and interpretation of the measured magnitudes hardly depends on the HW setup configured for the robot,
// it is needed to modify this class each time the number of hx711 modules or channels is modified.
// It is considered easier modify the class than to program a configurable class for all possible configurations

class ForceSensorsManager : public I_PeriodicTask
{
private:
	
	Multiple_HX711 multiple_hx711;
	
	// Readings
	int32_t Ax64ChannelValue_0;
	int32_t Bx32ChannelValue_0;
	int32_t Ax64ChannelValue_1;
	int32_t Bx32ChannelValue_1;
	int32_t Ax64ChannelValue_2;
	int32_t Bx32ChannelValue_2;
	int32_t Ax64ChannelValue_3;
	int32_t Bx32ChannelValue_3;
	
	// One filter per each measured magnitude
	ExpFilterPeakReject filter_ch0_Ax64;
	ExpFilterPeakReject filter_ch0_Bx32;
	ExpFilterPeakReject filter_ch1_Ax64;
	ExpFilterPeakReject filter_ch1_Bx32;
	ExpFilterPeakReject filter_ch2_Ax64;
	ExpFilterPeakReject filter_ch2_Bx32;
	ExpFilterPeakReject filter_ch3_Ax64;
	ExpFilterPeakReject filter_ch3_Bx32;
	
	Configuration::Configs::ForceSensors *config;
	
	double *calibration_LeftFoot_LeftFrontSensor;
	double *calibration_LeftFoot_RightFrontSensor;
	double *calibration_LeftFoot_LeftBackSensor;
	double *calibration_LeftFoot_RightBackSensor;
	double *calibration_RightFoot_LeftFrontSensor;
	double *calibration_RightFoot_RightFrontSensor;
	double *calibration_RightFoot_LeftBackSensor;
	double *calibration_RightFoot_RightBackSensor;

public:

	void assocConfig(Configs::ForceSensors &_config);

	void init();

	bool update();
	
	// Readings obtention in gr.
	int32_t getValue_LeftFoot_LeftFrontSensor();
	int32_t getValue_LeftFoot_RightFrontSensor();
	int32_t getValue_LeftFoot_LeftBackSensor();
	int32_t getValue_LeftFoot_RightBackSensor();
	int32_t getValue_RightFoot_LeftFrontSensor();
	int32_t getValue_RightFoot_RightFrontSensor();
	int32_t getValue_RightFoot_LeftBackSensor();
	int32_t getValue_RightFoot_RightBackSensor();
	
	uint32_t getLastElapsedTimeBetweenReadings();
	
	void tare_LeftFoot();
	void tare_RightFoot();
	
	// TODO: ZMP obtention
};

#endif

