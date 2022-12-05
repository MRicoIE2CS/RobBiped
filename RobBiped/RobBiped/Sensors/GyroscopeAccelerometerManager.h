
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
#include "../UserInput/Command.h"
#include "../Main/Configs.h"

using namespace Configuration;

class GyroscopeAccelerometerManager : public I_PeriodicTask
{
private:

	// Serial Commands pointer
 	Command* command;
	 
	Configuration::Configs::GyroscpeAccelerometer* config;

	MPU6050 mpu6050;
	
	// Raw values of accelerometer and gyroscope
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	
	// Scaled values of accelerometer and gyroscope
	float ax_m_s2, ay_m_s2, az_m_s2;
	float gx_deg_s, gy_deg_s, gz_deg_s;
	
	// Inclination angle of sensor, obtained from accelerometer
	float accel_ang_x;
	float accel_ang_y;
	
	// Rotation of sensor, obtained from gyroscope
	long tiempo_prev, dt;
	float ang_x, ang_y;
	float ang_x_prev, ang_y_prev;
	
	void getReadings();
	void unitsConversion();
	void calculateAccAngle();
	void complementaryFilter_Angle();
	void processReadings();
	
	// Calibration variables
	long f_ax,f_ay, f_az;	// Used in filter
	int p_ax, p_ay, p_az;	// Used in filter
	long f_gx,f_gy, f_gz;	// Used in filter
	int p_gx, p_gy, p_gz;	// Used in filter
	int counter=0;	// Used in filter
	int* ax_o = nullptr;	// Accelerometer offsets
	int* ay_o = nullptr;
	int* az_o = nullptr;
	int* gx_o = nullptr;	// Gyroscope offsets
	int* gy_o = nullptr;
	int* gz_o = nullptr;
	bool calibrate_first_run = true;
	void readOffsets();
	void printOffsets();
	void calibrate();

	void printValues();

public:

	void assocConfig(Configs::GyroscpeAccelerometer &_config);

	void init();

	bool update();
	
	void getValues(float* _ax_m_s2, float* _ay_m_s2, float* _az_m_s2, float* _gx_deg_s, float* _gy_deg_s, float* _gz_deg_s);
	
	void getValue_ax_m_s2(float* _ax_m_s2);
	float getValue_ax_m_s2();
	void getValue_ay_m_s2(float* _ay_m_s2);
	float getValue_ay_m_s2();
	void getValue_az_m_s2(float* _az_m_s2);
	float getValue_az_m_s2();
	
	void getValue_gx_deg_s(float* _gx_deg_s);
	float getValue_gx_deg_s();
	void getValue_gy_deg_s(float* _gy_deg_s);
	float getValue_gy_deg_s();
	void getValue_gz_deg_s(float* _gz_deg_s);
	float getValue_gz_deg_s();
};


#endif

