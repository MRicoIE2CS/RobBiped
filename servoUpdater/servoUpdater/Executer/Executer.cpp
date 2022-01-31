/*
 * Executer.cpp
 *
 * Created: 27/01/2022 11:28:19
 *  Author: MRICO
 */ 

#include "Executer.h"

void Executer::init(){
	
	signalGenerator_0.setExecutionPeriod(I_Task::execType::inMillis, 5);
	//signalGenerator_0.setExecutionPeriod(I_Task::execType::inMillis,2);	
	signalGenerator_0.configureSignal(SignalGenerator::SignalType::sine,500,1,0,0);
	signalGenerator_0.init();
// 	signalGenerator_1.setExecutionPeriod(I_Task::execType::inMillis,1);
// 	signalGenerator_1.configureSignal(SignalGenerator::SignalType::sine,500,1,0,250);
// 	signalGenerator_1.init();
	
	//servoUpdater.setExecutionPeriod(I_Task::execType::inMillis,2);	
	servoUpdater.setExecutionPeriod(I_Task::execType::inMillis,20);	
	servoUpdater.init();
	
}

void Executer::execution(){
	
	if (signalGenerator_0.getExecutionFlag()) {
		double nextAngle_0 = HALF_PI + 3*HALF_PI/4 * signalGenerator_0.generateTrajectory();
		//Serial.println("Angle: " + (String)nextAngle_0);
		servoUpdater.setAngleToServo(0,nextAngle_0);
		
// 		double nextAngle_1 = HALF_PI + 3*HALF_PI/4 * signalGenerator_1.generateTrajectory();
// 		servoUpdater.setAngleToServo(1,nextAngle_1);
	}
	
	if (servoUpdater.getExecutionFlag()) servoUpdater.update();
	
}