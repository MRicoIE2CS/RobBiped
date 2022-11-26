
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
#include "../UserInput/SerialCommand.h"
//#include "../Main/Configs.h"

class GyroscopeAccelerometerManager : public I_PeriodicTask
{
private:

	// Serial Commands pointer
 	SerialCommand* serialCommand;

	MPU6050 mu6050;
	
	// Raw values of accelerometer and gyroscope
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	
	// Scaled values of accelerometer and gyroscope
	float ax_m_s2, ay_m_s2, az_m_s2;
	float gx_deg_s, gy_deg_s, gz_deg_s;
	
	void getReadings();
	void getReadings_ISUnits();
	
	// Calibration variables
	long f_ax,f_ay, f_az;	// Used in filter
	int p_ax, p_ay, p_az;	// Used in filter
	long f_gx,f_gy, f_gz;	// Used in filter
	int p_gx, p_gy, p_gz;	// Used in filter
	int counter=0;	// Used in filter
	int ax_o,ay_o,az_o;	// Accelerometer offsets
	int gx_o,gy_o,gz_o;	// Gyroscope offsets
	bool calibrate_first_run = true;
	void readPreviousOffsets();
	void calibrate();

	void printValues();

public:

	//void assocConfig(Configs::ForceSensors &_config);

	void init();

	bool update();
	
	void getValues(float* ax_m_s2, float* ay_m_s2, float* az_m_s2, float* gx_deg_s, float* gy_deg_s, float* gz_deg_s);
	
	void getValue_ax_m_s2(float* ax_m_s2);
	void getValue_ay_m_s2(float* ay_m_s2);
	void getValue_az_m_s2(float* az_m_s2);
	
	void getValue_gx_deg_s(float* gx_deg_s);
	void getValue_gy_deg_s(float* gy_deg_s);
	void getValue_gz_deg_s(float* gz_deg_s);
};


#endif

