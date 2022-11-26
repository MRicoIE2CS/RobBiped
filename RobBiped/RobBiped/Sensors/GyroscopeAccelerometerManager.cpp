
/*
 * GyroscopeAccelerometerManager.cpp
 *
 * Created: 23/11/2022
 * Author: MRICO
 */ 

#include "GyroscopeAccelerometerManager.h"

void GyroscopeAccelerometerManager::init()
{
	Wire.begin();           // Initialize I2C
	mu6050.initialize();	// mu6050 initialize
	
	// Get SerialCommand singleton instance
	serialCommand = SerialCommand::getInstance();

	if (mu6050.testConnection()) Serial.println("mu6050 iniciado correctamente");
	else Serial.println("Error al iniciar el mu6050");
}

void GyroscopeAccelerometerManager::getReadings()
{
	// Read raw values
	mu6050.getAcceleration(&ax, &ay, &az);
	mu6050.getRotation(&gx, &gy, &gz);
}

void GyroscopeAccelerometerManager::getReadings_ISUnits()
{
	getReadings();
	// Convert raw values to IS units
	ax_m_s2 = ax * (9.81/16384.0);
	ay_m_s2 = ay * (9.81/16384.0);
	az_m_s2 = az * (9.81/16384.0);
	gx_deg_s = gx * (250.0/32768.0);
	gy_deg_s = gy * (250.0/32768.0);
	gz_deg_s = gz * (250.0/32768.0);
}

void GyroscopeAccelerometerManager::readPreviousOffsets()
{
	ax_o=mu6050.getXAccelOffset();
	ay_o=mu6050.getYAccelOffset();
	az_o=mu6050.getZAccelOffset();
	gx_o=mu6050.getXGyroOffset();
	gy_o=mu6050.getYGyroOffset();
	gz_o=mu6050.getZGyroOffset();
}

void GyroscopeAccelerometerManager::calibrate()
{
	getReadings_ISUnits();

	// Filtrar las lecturas
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
		if (p_ax>0) ax_o--;
		else {ax_o++;}
		if (p_ay>0) ay_o--;
		else {ay_o++;}
		if (p_az-16384>0) az_o--;
		else {az_o++;}
		
		mu6050.setXAccelOffset(ax_o);
		mu6050.setYAccelOffset(ay_o);
		mu6050.setZAccelOffset(az_o);

		// Calibrate gyroscope to 0º/s in every axis ()
		if (p_gx>0) gx_o--;
		else {gx_o++;}
		if (p_gy>0) gy_o--;
		else {gy_o++;}
		if (p_gz>0) gz_o--;
		else {gz_o++;}
		
		mu6050.setXGyroOffset(gx_o);
		mu6050.setYGyroOffset(gy_o);
		mu6050.setZGyroOffset(gz_o);
		
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
			readPreviousOffsets();
		}
		calibrate();

		if (serialCommand->commands.gyroacc_calibrate_off)
		{
			serialCommand->commands.gyroacc_calibrate_on = false;
			serialCommand->commands.gyroacc_calibrate_off = false;
			calibrate_first_run = true;
		}
	}
	else
	{
		getReadings_ISUnits();
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
	Serial.print("a[x y z](m/s2) g[x y z](deg/s):\t");
	Serial.print(ax_m_s2); Serial.print("\t");
	Serial.print(ay_m_s2); Serial.print("\t");
	Serial.print(az_m_s2); Serial.print("\t");
	Serial.print(gx_deg_s); Serial.print("\t");
	Serial.print(gy_deg_s); Serial.print("\t");
	Serial.println(gz_deg_s);
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