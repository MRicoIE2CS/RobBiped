/*
 * Executer.cpp
 *
 * Created: 27/01/2022 11:28:19
 *  Author: MRICO
 */ 

#include "Executer.h"

void Executer::init(){
	
	servoUpdater.configTask("servoUpdate", 4000,50);	//20ms execution period, priority of 50
	servoUpdater.init();
	
}

void Executer::execution(){
	
	if (servoUpdater.getExecutionFlag()) servoUpdater.update();
	
}