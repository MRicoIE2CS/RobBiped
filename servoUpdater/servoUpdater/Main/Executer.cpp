/*
 * Executer.cpp
 *
 * Created: 27/01/2022 11:28:19
 *  Author: MRICO
 */ 

#include "Executer.h"

void Executer::init(){
	
	setup();
	
	signalGenerator_0.setExecutionPeriod(I_PeriodicTask::execType::inMillis, 10);
	//signalGenerator_0.setExecutionPeriod(I_Task::execType::inMillis,2);	
	signalGenerator_0.configureSignal(SignalGenerator::SignalType::sine,500,1,0,0);
	signalGenerator_0.init();
// 	signalGenerator_1.setExecutionPeriod(I_Task::execType::inMillis,1);
// 	signalGenerator_1.configureSignal(SignalGenerator::SignalType::sine,500,1,0,250);
// 	signalGenerator_1.init();
	
	//servoUpdater.setExecutionPeriod(I_Task::execType::inMillis,2);	
	servoUpdater.setExecutionPeriod(I_PeriodicTask::execType::inMillis,20);	
	servoUpdater.init();
	
}

void Executer::setup(){
	
	IOs_setup();
}

void Executer::execution(){
	
	if (signalGenerator_0.getExecutionFlag()) {
		
		unsigned int pot1Val = analogRead(config.gpio.potentiometer1);
		Serial.println("pot1Val: " + (String)pot1Val);
		
		double nextAngle_0;
		double previousAngle;
		double expK = 0.9;
		if (pot1Val < 1000){
			nextAngle_0 = 0;
		}
		else if (pot1Val > 3000){
			nextAngle_0 = PI;
		}
		else {
			nextAngle_0 = (double)(pot1Val - 1000) / (double)2000 * PI;
		}
		//nextAngle_0 = previousAngle*expK + (1-expK)*nextAngle_0;
		//nextAngle_0 = HALF_PI + 3*HALF_PI/4 * signalGenerator_0.generateTrajectory();
		//Serial.println("Angle: " + (String)nextAngle_0);
		servoUpdater.setAngleToServo(0,nextAngle_0);
		previousAngle = nextAngle_0;
// 		double nextAngle_1 = HALF_PI + 3*HALF_PI/4 * signalGenerator_1.generateTrajectory();
// 		servoUpdater.setAngleToServo(1,nextAngle_1);
	}
	
	if (servoUpdater.getExecutionFlag()) servoUpdater.update();
	
}