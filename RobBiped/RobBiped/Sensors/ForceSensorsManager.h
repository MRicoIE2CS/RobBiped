
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
#include "../UserInput/Command.h"
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

	// Serial Commands pointer
	Command* command;
	
	Multiple_HX711 multiple_hx711;

	// Readings
	int32_t value_LeftFoot_LeftBack;
	int32_t value_LeftFoot_LeftFront;
	int32_t value_LeftFoot_RightBack;
	int32_t value_LeftFoot_RightFront;
	int32_t value_RightFoot_LeftBack;
	int32_t value_RightFoot_LeftFront;
	int32_t value_RightFoot_RightBack;
	int32_t value_RightFoot_RightFront;
	
	// One filter per each measured magnitude
	ExpFilterPeakReject filter_LeftFoot_LeftBack;
	ExpFilterPeakReject filter_LeftFoot_LeftFront;
	ExpFilterPeakReject filter_LeftFoot_RightBack;
	ExpFilterPeakReject filter_LeftFoot_RightFront;
	ExpFilterPeakReject filter_RightFoot_LeftBack;
	ExpFilterPeakReject filter_RightFoot_LeftFront;
	ExpFilterPeakReject filter_RightFoot_RightBack;
	ExpFilterPeakReject filter_RightFoot_RightFront;
	
	Configuration::Configs::ForceSensors *config;
	
	double *calibration_LeftFoot_LeftFrontSensor;
	double *calibration_LeftFoot_RightFrontSensor;
	double *calibration_LeftFoot_LeftBackSensor;
	double *calibration_LeftFoot_RightBackSensor;
	double *calibration_RightFoot_LeftFrontSensor;
	double *calibration_RightFoot_RightFrontSensor;
	double *calibration_RightFoot_LeftBackSensor;
	double *calibration_RightFoot_RightBackSensor;
	
	void printValues();

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
	void getValues_ZMP_leftFoot(int16_t& x_mm, int16_t& y_mm);
	void getValues_ZMP_rightFoot(int16_t& x_mm, int16_t& y_mm);
};

#endif

