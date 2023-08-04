/*
 * GyroscopeAccelerometerManager.h
 *
 * Copyright 2023 Mikel Rico Abajo (https://github.com/MRicoIE2CS)

 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */ 

#ifndef _GYROSCOPEACCELEROMETERMANAGER_h
#define _GYROSCOPEACCELEROMETERMANAGER_h

#include "arduino.h"

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
 	Command* command_;

	Configuration::Configs::GyroscpeAccelerometer* config_;

	MPU6050 mpu6050_;

	// Raw values of accelerometer and gyroscope
	int16_t ax_, ay_, az_;
	int16_t gx_, gy_, gz_;

	// Scaled values of accelerometer and gyroscope
	float ax_m_s2_, ay_m_s2_, az_m_s2_;
	float gx_deg_s_, gy_deg_s_, gz_deg_s_;

	// Inclination angle of sensor, obtained from accelerometer
	float accel_ang_x_;
	float accel_ang_y_;

	// Rotation of sensor, obtained from gyroscope
	int64_t tiempo_prev_, dt_;
	float ang_x_, ang_y_;
	float ang_x_prev_, ang_y_prev_;

	void get_readings();
	void units_conversion();
	void calculate_accelerometer_angle();
	void complementary_filter_for_angle();
	void process_readings();

	// Calibration variables
	int64_t f_ax_,f_ay_, f_az_;	// Used in filter
	int32_t p_ax_, p_ay_, p_az_;	// Used in filter
	int64_t f_gx_,f_gy_, f_gz_;	// Used in filter
	int32_t p_gx_, p_gy_, p_gz_;	// Used in filter
	int32_t counter_=0;	// Used in filter
	int16_t* ax_o_ = nullptr;	// Accelerometer offsets
	int16_t* ay_o_ = nullptr;
	int16_t* az_o_ = nullptr;
	int16_t* gx_o_ = nullptr;	// Gyroscope offsets
	int16_t* gy_o_ = nullptr;
	int16_t* gz_o_ = nullptr;
	bool calibrate_first_run_ = true;
	void read_offsets();
	void print_offsets();
	void calibrate();

	void print_values();

public:

	void assoc_config(Configs::GyroscpeAccelerometer &_config);

	void init();

	bool update();
	
	// TODO!: Check sign convention of all the getters

	void get_values(float& _ax_m_s2, float& _ay_m_s2, float& _az_m_s2, float& _gx_deg_s, float& _gy_deg_s, float& _gz_deg_s);

	void get_value_ax_m_s2(float& _ax_m_s2);
	float get_value_ax_m_s2();
	void get_value_ay_m_s2(float& _ay_m_s2);
	float get_value_ay_m_s2();
	void get_value_az_m_s2(float& _az_m_s2);
	float get_value_az_m_s2();

	void get_value_gx_deg_s(float& _gx_deg_s);
	float get_value_gx_deg_s();
	void get_value_gy_deg_s(float& _gy_deg_s);
	float get_value_gy_deg_s();
	void get_value_gz_deg_s(float& _gz_deg_s);
	float get_value_gz_deg_s();

	/*
	*  @fn void get_value_angle_z_pitch_deg(float* _ang_pitch);
	*  @fn float get_value_angle_z_pitch_deg();
	*  @brief Getter for the inclination angle of the z axis of the sensor, relative to the vertical
	*  axis (of the world), over the sagittal plane. Pitch rotation (rotation over the local X axis of the sensor).
	*
	*  @param[out] _ang_pitch Pitch inclination angle.
	*/
	void get_value_angle_z_pitch_deg(float& _ang_pitch);
	float get_value_angle_z_pitch_deg();

	/*
	*  @fn void get_value_angle_z_roll_deg(float* _ang_roll);
	*  @fn float get_value_angle_z_roll_deg();
	*  @brief Getter for the inclination angle of the z axis of the sensor, relative to the vertical
	*  axis (of the world), over the frontal plane. Roll rotation (rotation over the local Y axis of the sensor).
	*
	*  @param[out] _ang_roll Roll inclination angle.
	*/
	void get_value_angle_z_roll_deg(float& _ang_roll);
	float get_value_angle_z_roll_deg();

	/*
	*  @fn void get_value_angle_z_pitch_rad(float* _ang_pitch);
	*  @fn float get_value_angle_z_pitch_rad();
	*  @brief Getter for the inclination angle of the z axis of the sensor, relative to the vertical
	*  axis (of the world), over the sagittal plane. Pitch rotation (rotation over the local X axis of the sensor).
	*
	*  @param[out] _ang_pitch Pitch inclination angle.
	*/
	void get_value_angle_z_pitch_rad(float& _ang_pitch);
	float get_value_angle_z_pitch_rad();

	/*
	*  @fn void get_value_angle_z_roll_rad(float* _ang_roll);
	*  @fn float get_value_angle_z_roll_rad();
	*  @brief Getter for the inclination angle of the z axis of the sensor, relative to the vertical
	*  axis (of the world), over the frontal plane. Roll rotation (rotation over the local Y axis of the sensor).
	*
	*  @param[out] _ang_roll Roll inclination angle.
	*/
	void get_value_angle_z_roll_rad(float& _ang_roll);
	float get_value_angle_z_roll_rad();
};

#endif
