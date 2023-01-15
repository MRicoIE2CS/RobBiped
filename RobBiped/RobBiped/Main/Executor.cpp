/*
 * Executor.cpp
 *
 * Created: 27/01/2022 11:28:19
 *  Author: MRICO
 */ 

#include "Executor.h"

void Executor::init()
{
	setup();
	
	//______TASKS CONFIGURATION_____//
	
	userInput.setExecutionPeriod(I_PeriodicTask::execType::inMillis, 5);
	
	signalGenerator_0.setExecutionPeriod(I_PeriodicTask::execType::inMillis, 20);
	signalGenerator_0.configureSignal(SignalGenerator::SignalType::sine,500,1,0,0);
	signalGenerator_0.init();
	
	servoUpdater.setExecutionPeriod(I_PeriodicTask::execType::inMillis,20);	
	servoUpdater.init();
	
	forceSensorsManager.setExecutionPeriod(I_PeriodicTask::execType::inMillis, 5);
	forceSensorsManager.init();
	
	gyroscopeAccelerometerManager.setExecutionPeriod(I_PeriodicTask::execType::inMillis, 4);
	gyroscopeAccelerometerManager.init();
	
	// END TASKS CONFIGURATION
}

void Executor::inputs()
{
	if (userInput.getExecutionFlag()) userInput.update();
	
	if (gyroscopeAccelerometerManager.getExecutionFlag())
	{
		bool updated = gyroscopeAccelerometerManager.update();
	}
	
	if (forceSensorsManager.getExecutionFlag())
	{
		bool updated = forceSensorsManager.update();
	}
}

void Executor::mainExecution()
{
	
	
	
}

void Executor::outputs()
{
	if (servoUpdater.getExecutionFlag())
	{
// 		// INIT Example of using potentiometer to command servos
// 		uint16_t pot1Val = userInput.getAnalogValue(UserInput::AnalogInputList::potentiometer2);
// 		
// 		double readingAngle_0;
// 		double max_val = 0.2;
// 		double min_val = -0.2;
// 		double ampl = max_val - min_val;
// 		if (pot1Val < 1000){
// 			readingAngle_0 = 0;
// 		}
// 		else if (pot1Val > 3000){
// 			readingAngle_0 = ampl;
// 		}
// 		else {
// 			readingAngle_0 = (double)(pot1Val - 1000) / (double)2000 * ampl;
// 		}
// 		double nextAngle_0 = readingAngle_0 - ampl / 2;
		double nextAngle_0 = 0.0;
		// END Example
		
		// Servo setpoint assignation
		servoUpdater.setAngleToServo(0,nextAngle_0);
		servoUpdater.setAngleToServo(1,nextAngle_0);
		servoUpdater.setAngleToServo(2,nextAngle_0);
		servoUpdater.setAngleToServo(3,nextAngle_0);
		servoUpdater.setAngleToServo(4,nextAngle_0);
		servoUpdater.setAngleToServo(5,nextAngle_0);
		servoUpdater.setAngleToServo(6,nextAngle_0);
		servoUpdater.setAngleToServo(7,nextAngle_0);
		servoUpdater.setAngleToServo(8,nextAngle_0);
		servoUpdater.setAngleToServo(9,nextAngle_0);
		servoUpdater.setAngleToServo(10,nextAngle_0);
		servoUpdater.setAngleToServo(11,nextAngle_0);
		servoUpdater.setAngleToServo(12,nextAngle_0);
		servoUpdater.setAngleToServo(13,nextAngle_0);
		servoUpdater.setAngleToServo(14,nextAngle_0);
		servoUpdater.setAngleToServo(15,nextAngle_0);
		
		// Servo setpoint command
		servoUpdater.update(userInput);
	}
}

void Executor::execution()
{
	inputs();
	
	mainExecution();
	
	outputs();
}