
/*
 * GyroscopeAccelerometerManager.cpp
 *
 * Created: 23/11/2022
 * Author: MRICO
 */ 

#include "GyroscopeAccelerometerManager.h"

void GyroscopeAccelerometerManager::assocConfig(Configs::GyroscpeAccelerometer &_config){
	config = &_config;
}

void GyroscopeAccelerometerManager::init()
{
	Wire.begin();           // Initialize I2C
	mpu5060.initialize();	// mpu5060 initialize
	
	// Set storaged offsets
	ax_o = &(config->offsets.ax_o);
	ay_o = &(config->offsets.ay_o);
	az_o = &(config->offsets.az_o);
	gx_o = &(config->offsets.gx_o);
	gy_o = &(config->offsets.gy_o);
	gz_o = &(config->offsets.gz_o);
	mpu5060.setXAccelOffset(*ax_o);
	mpu5060.setYAccelOffset(*ay_o);
	mpu5060.setZAccelOffset(*az_o);
	mpu5060.setXGyroOffset(*gx_o);
	mpu5060.setYGyroOffset(*gy_o);
	mpu5060.setZGyroOffset(*gz_o);
	
	// Get SerialCommand singleton instance
	serialCommand = SerialCommand::getInstance();

	if (mpu5060.testConnection()) Serial.println("mpu5060 iniciado correctamente");
	else Serial.println("Error al iniciar el mpu5060");
}

void GyroscopeAccelerometerManager::getReadings()
{
	// Read raw values
	mpu5060.getAcceleration(&ax, &ay, &az);
	mpu5060.getRotation(&gx, &gy, &gz);
}

void GyroscopeAccelerometerManager::unitsConversion()
{
	// Convert raw values to IS units
	ax_m_s2 = ax * (9.81/16384.0);		// +-2g scale -> +-32768
	ay_m_s2 = ay * (9.81/16384.0);
	az_m_s2 = az * (9.81/16384.0);
	gx_deg_s = gx * (250.0/32768.0);	// +-250deg/s scale -> +-32768
	gy_deg_s = gy * (250.0/32768.0);
	gz_deg_s = gz * (250.0/32768.0);
}

void GyroscopeAccelerometerManager::calculateAccAngle()
{
	// Calculate inclination angles
	accel_ang_x = atan( ax/ sqrt( pow(ay,2) + pow(az,2) ) ) * (180.0/3.14);
	accel_ang_y = atan( ay/ sqrt( pow(ax,2) + pow(az,2) ) ) * (180.0/3.14);
}

void GyroscopeAccelerometerManager::calculateGyroRotation()
{
	auto currentMillis = millis();
	dt = currentMillis - tiempo_prev;
	tiempo_prev = currentMillis;
	
	girosc_ang_x = gx_deg_s * dt / 1000.0 + girosc_ang_x_prev;
	girosc_ang_y = gy_deg_s * dt / 1000.0 + girosc_ang_y_prev;

	girosc_ang_x_prev = girosc_ang_x;
	girosc_ang_y_prev = girosc_ang_y;
}

void GyroscopeAccelerometerManager::processReadings()
{
	unitsConversion();
	calculateAccAngle();
	calculateGyroRotation();
}

void GyroscopeAccelerometerManager::readOffsets()
{
	*ax_o=mpu5060.getXAccelOffset();
	*ay_o=mpu5060.getYAccelOffset();
	*az_o=mpu5060.getZAccelOffset();
	*gx_o=mpu5060.getXGyroOffset();
	*gy_o=mpu5060.getYGyroOffset();
	*gz_o=mpu5060.getZGyroOffset();
}

void GyroscopeAccelerometerManager::printOffsets()
{
	Serial.println("ax_o = " + (String)*ax_o);
	Serial.println("ay_o = " + (String)*ay_o);
	Serial.println("az_o = " + (String)*az_o);
	Serial.println("gx_o = " + (String)*gx_o);
	Serial.println("gy_o = " + (String)*gy_o);
	Serial.println("gz_o = " + (String)*gz_o);
}

void GyroscopeAccelerometerManager::calibrate()
{
	getReadings();

	// Filter readings (kind of moving average of 2^5 samples for accelerometer and 2^3 for gyroscope)
	f_ax = f_ax-(f_ax>>5)+ax;
	p_ax = f_ax>>5;

	f_ay = f_ay-(f_ay>>5)+ay;
	p_ay = f_ay>>5;

	f_az = f_az-(f_az>>5)+az;
	p_az = f_az>>5;

	f_gx = f_gx-(f_gx>>3)+gx;
	p_gx = f_gx>>3;

	f_gy = f_gy-(f_gy>>3)+gy;
	p_gy = f_gy>>3;

	f_gz = f_gz-(f_gz>>3)+gz;
	p_gz = f_gz>>3;

	// Correct offset every 100 readings
	if (counter==100){

		// Calibrate accelerometer for 1g in z axis (adjust offset)
		if (p_ax>0) *ax_o--;
		else {*ax_o++;}
		if (p_ay>0) *ay_o--;
		else {*ay_o++;}
		if (p_az-16384>0) *az_o--;
		else {*az_o++;}
		
		mpu5060.setXAccelOffset(*ax_o);
		mpu5060.setYAccelOffset(*ay_o);
		mpu5060.setZAccelOffset(*az_o);

		// Calibrate gyroscope to 0º/s in every axis ()
		if (p_gx>0) *gx_o--;
		else {*gx_o++;}
		if (p_gy>0) *gy_o--;
		else {*gy_o++;}
		if (p_gz>0) *gz_o--;
		else {*gz_o++;}
		
		mpu5060.setXGyroOffset(*gx_o);
		mpu5060.setYGyroOffset(*gy_o);
		mpu5060.setZGyroOffset(*gz_o);
		
		counter = 0;
	}
	counter++;
}

bool GyroscopeAccelerometerManager::update()
{
	
	if (serialCommand->commands.gyroacc_calibrate_on)
	{
		if (calibrate_first_run){
			calibrate_first_run = false;
			//readOffsets(); this here reads the offsets from chip, which are not persistant values
		}
		calibrate();

		if (serialCommand->commands.gyroacc_calibrate_off)
		{
			serialCommand->commands.gyroacc_calibrate_on = false;
			serialCommand->commands.gyroacc_calibrate_off = false;
			calibrate_first_run = true;
			printOffsets();
		}
	}
	else
	{
		getReadings();
		processReadings();
	}
	
	if (serialCommand->commands.gyroacc_debug_on)
	{
		printValues();
		if (serialCommand->commands.gyroacc_debug_off)
		{
			serialCommand->commands.gyroacc_debug_on = false;
			serialCommand->commands.gyroacc_debug_off = false;
		}
	}
}

void GyroscopeAccelerometerManager::printValues()
{
	Serial.println("_________________________________");
	Serial.print("a[x y z](m/s2) g[x y z](deg/s):\t");
	Serial.print(ax_m_s2); Serial.print("\t");
	Serial.print(ay_m_s2); Serial.print("\t");
	Serial.print(az_m_s2); Serial.print("\t");
	Serial.print(gx_deg_s); Serial.print("\t");
	Serial.print(gy_deg_s); Serial.print("\t");
	Serial.println(gz_deg_s);
	
	Serial.print("X inclination: ");
	Serial.print(accel_ang_x);
	Serial.print("Y inclination:");
	Serial.println(accel_ang_y);
	
	Serial.print("X rotation:  ");
	Serial.print(girosc_ang_x);
	Serial.print("Y rotation: ");
	Serial.println(girosc_ang_y);
}


// TODO: implement
// void getValues(float* ax_m_s2, float* ay_m_s2, float* az_m_s2, float* gx_deg_s, float* gy_deg_s, float* gz_deg_s);
// 
// void getValue_ax_m_s2(float* ax_m_s2);
// void getValue_ay_m_s2(float* ay_m_s2);
// void getValue_az_m_s2(float* az_m_s2);
// 
// void getValue_gx_deg_s(float* gx_deg_s);
// void getValue_gy_deg_s(float* gy_deg_s);
// void getValue_gz_deg_s(float* gz_deg_s);