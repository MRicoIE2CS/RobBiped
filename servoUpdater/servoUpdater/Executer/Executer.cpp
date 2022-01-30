/*
 * Executer.cpp
 *
 * Created: 27/01/2022 11:28:19
 *  Author: MRICO
 */ 

#include "Executer.h"

void Executer::init(){
	
	trajectoryGenerator.configTask("trajectoryGenerator",50,50);
	
	servoUpdater.configTask("servoUpdate", 250,50);	//250ms execution period, priority of 50
	servoUpdater.init();
	
}

void Executer::execution(){
	
	if (trajectoryGenerator.getExecutionFlag()) trajectoryGenerator.generateTrajectory();
	
	if (servoUpdater.getExecutionFlag()) servoUpdater.update();
	
}