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
	
// LEFT FOOT - ROLL
	jointInitializer.invertAngleSign(false);
	jointInitializer.calibration_setZero(false, 1.20);
	jointInitializer.calibration_setMinAngle(false, -0.2);
	jointInitializer.calibration_setMaxAngle(false, 0.4);
	PCA9685_1_servoMap[0] = jointInitializer;
// LEFT FOOT - PITCH	
	jointInitializer.invertAngleSign(false);
	jointInitializer.calibration_setZero(false, 1.28);
	jointInitializer.calibration_setMinAngle(false, -0.6);
	jointInitializer.calibration_setMaxAngle(false, 0.8);
	PCA9685_1_servoMap[1] = jointInitializer;
	
// LEFT KNEE
	jointInitializer.invertAngleSign(true);
	jointInitializer.calibration_setZero(false, 1.57);
	jointInitializer.calibration_setMinAngle(false, -0.2);
	jointInitializer.calibration_setMaxAngle(false, 1.5);
	PCA9685_1_servoMap[2] = jointInitializer;

// LEFT HIP - PITCH
	jointInitializer.invertAngleSign(false);
	jointInitializer.calibration_setZero(false, 1.79);
	jointInitializer.calibration_setMinAngle(false, -0.45);
	jointInitializer.calibration_setMaxAngle(false, 1);
	PCA9685_1_servoMap[3] = jointInitializer;
// LEFT HIP - ROLL
	jointInitializer.invertAngleSign(false);
	jointInitializer.calibration_setZero(false, 1.75);
	jointInitializer.calibration_setMinAngle(false, -0.1);
	jointInitializer.calibration_setMaxAngle(false, 0.5);
	PCA9685_1_servoMap[4] = jointInitializer;
	

// LEFT SHOULDER
	jointInitializer.invertAngleSign(false);
	jointInitializer.calibration_setZero(false, 1.41);
	jointInitializer.calibration_setMinAngle(false, -0.45);
	jointInitializer.calibration_setMaxAngle(false, 0.45);
	PCA9685_1_servoMap[5] = jointInitializer;
// LEFT SHOULDER
	jointInitializer.invertAngleSign(false);
	jointInitializer.calibration_setZero(false, 1.37);
	jointInitializer.calibration_setMinAngle(false, -0.45);
	jointInitializer.calibration_setMaxAngle(false, 0.45);
	PCA9685_1_servoMap[6] = jointInitializer;
	
// UNUSED
	jointInitializer.cleanCalibrationValues();
	PCA9685_1_servoMap[7] = jointInitializer;
	PCA9685_1_servoMap[8] = jointInitializer;

// RIGHT SHOULDER
	jointInitializer.invertAngleSign(true);
	jointInitializer.calibration_setZero(false, 1.27);
	jointInitializer.calibration_setMinAngle(false, -0.45);
	jointInitializer.calibration_setMaxAngle(false, 0.45);
	PCA9685_1_servoMap[9] = jointInitializer;
// RIGHT SHOULDER
	jointInitializer.invertAngleSign(true);
	jointInitializer.calibration_setZero(false, 1.27);
	jointInitializer.calibration_setMinAngle(false, -0.45);
	jointInitializer.calibration_setMaxAngle(false, 0.45);
	PCA9685_1_servoMap[10] = jointInitializer;

// RIGHT HIP - ROLL
	jointInitializer.invertAngleSign(true);
	jointInitializer.calibration_setZero(false, 1.46);
	jointInitializer.calibration_setMinAngle(false, -0.45);
	jointInitializer.calibration_setMaxAngle(false, 0.45);
	PCA9685_1_servoMap[11] = jointInitializer;
// RIGHT HIP - PITCH
	jointInitializer.invertAngleSign(true);
	jointInitializer.calibration_setZero(false, 1.64);
	jointInitializer.calibration_setMinAngle(false, -0.45);
	jointInitializer.calibration_setMaxAngle(false, 0.45);
	PCA9685_1_servoMap[12] = jointInitializer;
	
	
// ESTROPEADO!!!!!
// RIGHT KNEE
	jointInitializer.invertAngleSign(false);
	jointInitializer.calibration_setZero(false, 1.64);
	jointInitializer.calibration_setMinAngle(false, -0.45);
	jointInitializer.calibration_setMaxAngle(false, 0.45);
	jointInitializer.cleanCalibrationValues();
	PCA9685_1_servoMap[13] = jointInitializer;
	
	
//RIGHT FOOT - ROLL
	jointInitializer.invertAngleSign(true);
	jointInitializer.calibration_setZero(false, 1.38);
	jointInitializer.calibration_setMinAngle(false, -0.8);
	jointInitializer.calibration_setMaxAngle(false, 0.6);
	PCA9685_1_servoMap[14] = jointInitializer;
//RIGHT FOOT - PITCH
	jointInitializer.invertAngleSign(true);
	jointInitializer.calibration_setZero(false, 1.67);
	jointInitializer.calibration_setMinAngle(false, -0.4);
	jointInitializer.calibration_setMaxAngle(false, 0.2);
	PCA9685_1_servoMap[15] = jointInitializer;
}