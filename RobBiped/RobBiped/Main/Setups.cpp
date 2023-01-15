/*
 * Setups.cpp
 *
 * Created: 13/02/2022 20:04:17
 *  Author: MRICO
 */ 

#include "Executor.h"

void Executor::setup()
{
	associations();
}

void Executor::associations()
{
	userInput.assocGPIO(config.userInputPins);
	//TODO: Associate buttons to servo updater calibration
	//servoUpdater.assocButtons(config.gpio.thinButton1, config.gpio.thinButton2);
	forceSensorsManager.assocConfig(config.forceSensors);
	gyroscopeAccelerometerManager.assocConfig(config.gyroAcc);
}