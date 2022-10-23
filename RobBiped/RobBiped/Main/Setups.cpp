/*
 * Setups.cpp
 *
 * Created: 13/02/2022 20:04:17
 *  Author: MRICO
 */ 

#include "Executer.h"

void Executer::setup()
{
	associations();
}

void Executer::associations()
{
	userInput.assocGPIO(config.userInputPins);
	//TODO: Associate buttons to servo updater calibration
	//servoUpdater.assocButtons(config.gpio.thinButton1, config.gpio.thinButton2);
	forceSensorsManager.assocConfig(config.forceSensors);
}