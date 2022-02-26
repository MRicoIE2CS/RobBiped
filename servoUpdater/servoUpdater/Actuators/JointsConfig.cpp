/*
 * ServosConfig.cpp
 *
 * Created: 26/02/2022 16:46:36
 *  Author: MRICO
 */ 

#include "JointsManager.h"

void JointsManager::jointsConfig(){
	
	Joint jointInitializer;
	
	jointInitializer.cleanCalibrationValues();
	//jointInitializer.invertAngleSign(false);
	//jointInitializer.calibration_setZero(false, 0.0);
	//jointInitializer.calibration_setMinAngle(false, -HALF_PI);
	//jointInitializer.calibration_setMaxAngle(false, HALF_PI);
	jointInitializer.setAngleTarget_rad(0.0);
	PCA9685_1_servoMap[0] = jointInitializer;
	
	// 	jointInitializer.setAngleTarget(0);
	// 	PCA9685_1_servoMap[1] = jointInitializer;
}