
/*
 * GyroscopeAccelerometerManager.h
 *
 * Created: 23/11/2022
 * Author: MRICO
 */ 

#ifndef _GYROSCOPEACCELEROMETERMANAGER_h
#define _GYROSCOPEACCELEROMETERMANAGER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// I2C libraries to control the MPU6050
// MPU6050.h needs I2Cdev.h, I2Cdev.h needs Wire.h
#include <Wire.h>
#include "GyroAcc/MPU6050/I2Cdev.h"
#include "GyroAcc/MPU6050/MPU6050.h"

#include "../Main/I_PeriodicTask.h"
//#include "../Main/Configs.h"

class GyroscopeAccelerometerManager : public I_PeriodicTask
{
private:

	MPU6050 sensor;
	
	// Raw values of accelerometer and gyroscope
	int16_t ax, ay, az;
	int16_t gx, gy, gz;

public:

	//void assocConfig(Configs::ForceSensors &_config);

	void init();

	bool update();
};


#endif

