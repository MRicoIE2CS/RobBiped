/*
 * Executer.cpp
 *
 * Created: 27/01/2022 11:28:19
 *  Author: MRICO
 */ 

#include "Executer.h"

void Executer::init(){
	
	setup();
	
	//______TASKS CONFIGURATION_____//
	
	userInput.setExecutionPeriod(I_PeriodicTask::execType::inMillis, 5);
	
	signalGenerator_0.setExecutionPeriod(I_PeriodicTask::execType::inMillis, 20);
	signalGenerator_0.configureSignal(SignalGenerator::SignalType::sine,500,1,0,0);
	signalGenerator_0.init();
	
	servoUpdater.setExecutionPeriod(I_PeriodicTask::execType::inMillis,20);	
	servoUpdater.init();
	
	// END TASKS CONFIGURATION
	
	
}

void Executer::execution(){
	
	// INPUTS:
	
	if (userInput.getExecutionFlag()) userInput.update();
	
	// MAIN EXECUTION:
	
	//if (userInput.getDigitalValue(UserInput::DigitalInputList::squareButton)) servoUpdater.changeState();	// run/sleep to servos
	
	if (signalGenerator_0.getExecutionFlag()) {
		
		uint16_t pot1Val = userInput.getAnalogValue(UserInput::AnalogInputList::potentiometer2);
		
		double readingAngle_0;
		if (pot1Val < 1000){
			readingAngle_0 = 0;
		}
		else if (pot1Val > 3000){
			readingAngle_0 = PI;
		}
		else {
			readingAngle_0 = (double)(pot1Val - 1000) / (double)2000 * PI;
		}
		double nextAngle_0 = readingAngle_0 - HALF_PI;
		nextAngle_0 = 0.0;
		//nextAngle_0 = HALF_PI + 3*HALF_PI/4 * signalGenerator_0.generateTrajectory();
		servoUpdater.setAngleToServo(0,nextAngle_0);
		servoUpdater.setAngleToServo(1,nextAngle_0);
		servoUpdater.setAngleToServo(2,nextAngle_0);
		servoUpdater.setAngleToServo(3,nextAngle_0);
	}
	
	// OUTPUTS:
	
	if (servoUpdater.getExecutionFlag()) servoUpdater.update(userInput);
	
}