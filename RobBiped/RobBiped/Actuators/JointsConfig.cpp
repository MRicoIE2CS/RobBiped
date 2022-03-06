/*
 * ServosConfig.cpp
 *
 * Created: 26/02/2022 16:46:36
 *  Author: MRICO
 */ 

#include "JointsManager.h"

void JointsManager::jointsConfig(){
	
	Joint jointInitializer;
	
	//jointInitializer.cleanCalibrationValues();
	
//LEFT FOOT - ROOL:
	jointInitializer.invertAngleSign(true);
	jointInitializer.calibration_setZero(false, 1.74);
	jointInitializer.calibration_setMinAngle(false, -0.6);
	jointInitializer.calibration_setMaxAngle(false, 0.5);
	PCA9685_1_servoMap[0] = jointInitializer;
//LEFT FOOT - PITCH:	
	jointInitializer.invertAngleSign(true);
	jointInitializer.calibration_setZero(false, 1.48);
	jointInitializer.calibration_setMinAngle(false, -0.45);
	jointInitializer.calibration_setMaxAngle(false, 0.9);
	PCA9685_1_servoMap[1] = jointInitializer;
//RIGHT FOOT - ROOL:
	jointInitializer.invertAngleSign(false);
	jointInitializer.calibration_setZero(false, 1.28);
	jointInitializer.calibration_setMinAngle(false, -0.6);
	jointInitializer.calibration_setMaxAngle(false, 0.5);
	PCA9685_1_servoMap[2] = jointInitializer;
//RIGHT FOOT - PITCH:	
	jointInitializer.calibration_setZero(false, 1.16);
	jointInitializer.calibration_setMinAngle(false, -0.9);
	jointInitializer.calibration_setMaxAngle(false, 0.45);
	PCA9685_1_servoMap[3] = jointInitializer;
	
	jointInitializer.cleanCalibrationValues();
	PCA9685_1_servoMap[4] = jointInitializer;
	PCA9685_1_servoMap[5] = jointInitializer;
	PCA9685_1_servoMap[6] = jointInitializer;
	PCA9685_1_servoMap[7] = jointInitializer;
	PCA9685_1_servoMap[8] = jointInitializer;
	PCA9685_1_servoMap[9] = jointInitializer;
	PCA9685_1_servoMap[10] = jointInitializer;
	PCA9685_1_servoMap[11] = jointInitializer;
	PCA9685_1_servoMap[12] = jointInitializer;
	PCA9685_1_servoMap[13] = jointInitializer;
	PCA9685_1_servoMap[14] = jointInitializer;
	PCA9685_1_servoMap[15] = jointInitializer;
}