/*
 * GyroscopeAccelerometerManager.cpp
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

#include "GyroscopeAccelerometerManager.h"

void GyroscopeAccelerometerManager::assoc_config(Configs::GyroscpeAccelerometer &_config){
	config_ = &_config;
}

void GyroscopeAccelerometerManager::init()
{
	Wire.begin();           // Initialize I2C
	
	mpu6050_.initialize();	// mpu6050 initialize
	
	// Set stored offsets
	ax_o_ = &(config_->offsets.ax_o);
	ay_o_ = &(config_->offsets.ay_o);
	az_o_ = &(config_->offsets.az_o);
	gx_o_ = &(config_->offsets.gx_o);
	gy_o_ = &(config_->offsets.gy_o);
	gz_o_ = &(config_->offsets.gz_o);
	mpu6050_.setXAccelOffset(*ax_o_);
	mpu6050_.setYAccelOffset(*ay_o_);
	mpu6050_.setZAccelOffset(*az_o_);
	mpu6050_.setXGyroOffset(*gx_o_);
	mpu6050_.setYGyroOffset(*gy_o_);
	mpu6050_.setZGyroOffset(*gz_o_);

	ax_m_s2_filtered_.set_time_constant(config_->filter_time_constant_ms);
	ay_m_s2_filtered_.set_time_constant(config_->filter_time_constant_ms);
	az_m_s2_filtered_.set_time_constant(config_->filter_time_constant_ms);

	Kf = config_->Kf_complement_filter;
	
	// Get Command singleton instance
	command_ = Command::get_instance();

	if (mpu6050_.testConnection()) Serial.println("mpu6050 iniciado correctamente");
	else Serial.println("Error al iniciar el mpu6050");
}

void GyroscopeAccelerometerManager::get_readings()
{
	// Read raw values
	mpu6050_.getAcceleration(&ax_, &ay_, &az_);

	mpu6050_.getRotation(&gx_, &gy_, &gz_);
}

void GyroscopeAccelerometerManager::units_conversion()
{
	// Convert raw values to IS units
	ax_m_s2_ = ax_ * (9.807/16384.0);		// +-2g scale -> +-32768
	ay_m_s2_ = ay_ * (9.807/16384.0);
	az_m_s2_ = az_ * (9.807/16384.0);

	gx_deg_s_ = gx_ * (250.0/32768.0);	// +-250deg/s scale -> +-32768
	gy_deg_s_ = gy_ * (250.0/32768.0);
	gz_deg_s_ = gz_ * (250.0/32768.0);

	if (0.0 == gx_deg_s_ && 0.0 == gy_deg_s_ && 0.0 == gz_deg_s_) Serial.println("NOT GETTING GYROSCOPE READINGS!!!");
}

void GyroscopeAccelerometerManager::calculate_accelerometer_angle()
{
	// Calculate inclination angles
	double sum_of_squares_yz = pow(ay_m_s2_,2) + pow(az_m_s2_,2);
	double sum_of_squares_xz = pow(ax_m_s2_,2) + pow(az_m_s2_,2);
	if ( (0.0 == sum_of_squares_yz) || (0.0 == sum_of_squares_xz) ) return; // Avoid division by zero or NaN
	accel_rotang_x_ = atan( ay_m_s2_/ sqrt( sum_of_squares_xz ) ) * (180.0/PI);	// In degrees
	accel_rotang_y_ = atan( -ax_m_s2_/ sqrt( sum_of_squares_yz ) ) * (180.0/PI);	// In degrees
}

void GyroscopeAccelerometerManager::complementary_filter_for_angle()
{
	uint32_t currentMillis = millis();
	dt_ = currentMillis - tiempo_prev_;
	tiempo_prev_ = currentMillis;

	rotang_deg_x_ = Kf * ((gx_deg_s_) * (float)dt_ / 1000.0 + rotang_deg_x_prev_) + (1-Kf) * accel_rotang_x_;
	rotang_deg_y_ = Kf * ((gy_deg_s_) * (float)dt_ / 1000.0 + rotang_deg_y_prev_) + (1-Kf) * accel_rotang_y_;

	rotang_deg_x_prev_ = rotang_deg_x_;
	rotang_deg_y_prev_ = rotang_deg_y_;
}

void GyroscopeAccelerometerManager::gain_adjust()
{
	// Empirical adjustments
	// TODO: Get this out, to the configurations file
	if (rotang_deg_x_ > 0.0)
	{
		ax_m_s2_ = ax_m_s2_ * (9.807/9.09);
	}
	if (rotang_deg_x_ < 0.0)
	{
		ax_m_s2_ = ax_m_s2_ * (9.807/10.45);
	}
	if (rotang_deg_y_ < 0.0)
	{
		ay_m_s2_ = ay_m_s2_ * (9.807/9.92);
	}
}

void GyroscopeAccelerometerManager::filter_acc_values()
{
	ax_m_s2_filtered_.filter(ax_m_s2_);
	ay_m_s2_filtered_.filter(ay_m_s2_);
	az_m_s2_filtered_.filter(az_m_s2_);
}

void GyroscopeAccelerometerManager::process_readings()
{
	units_conversion();
	calculate_accelerometer_angle();
	complementary_filter_for_angle();
	gain_adjust();
	filter_acc_values();
}

void GyroscopeAccelerometerManager::read_offsets()
{
	*ax_o_=mpu6050_.getXAccelOffset();
	*ay_o_=mpu6050_.getYAccelOffset();
	*az_o_=mpu6050_.getZAccelOffset();
	*gx_o_=mpu6050_.getXGyroOffset();
	*gy_o_=mpu6050_.getYGyroOffset();
	*gz_o_=mpu6050_.getZGyroOffset();
}

void GyroscopeAccelerometerManager::print_offsets()
{
	Serial.println("ax_o = " + (String)*ax_o_);
	Serial.println("ay_o = " + (String)*ay_o_);
	Serial.println("az_o = " + (String)*az_o_);
	Serial.println("gx_o = " + (String)*gx_o_);
	Serial.println("gy_o = " + (String)*gy_o_);
	Serial.println("gz_o = " + (String)*gz_o_);
}

void GyroscopeAccelerometerManager::calibrate()
{
	get_readings();

	// Filter readings (kind of moving average of 2^5 samples for accelerometer and 2^3 for gyroscope)
	f_ax_ = f_ax_-(f_ax_>>5)+ax_;
	p_ax_ = f_ax_>>5;

	f_ay_ = f_ay_-(f_ay_>>5)+ay_;
	p_ay_ = f_ay_>>5;

	f_az_ = f_az_-(f_az_>>5)+az_;
	p_az_ = f_az_>>5;

	f_gx_ = f_gx_-(f_gx_>>3)+gx_;
	p_gx_ = f_gx_>>3;

	f_gy_ = f_gy_-(f_gy_>>3)+gy_;
	p_gy_ = f_gy_>>3;

	f_gz_ = f_gz_-(f_gz_>>3)+gz_;
	p_gz_ = f_gz_>>3;

	// Correct offset every 100 readings
	if (counter_==100){

		// Calibrate accelerometer for 1g in z axis (adjust offset)
		if (p_ax_>0) (*ax_o_)--;
		else {(*ax_o_)++;}
		if (p_ay_>0) (*ay_o_)--;
		else {(*ay_o_)++;}
		if (p_az_-16384>0) (*az_o_)--;
		else {(*az_o_)++;}
		
		mpu6050_.setXAccelOffset(*ax_o_);
		mpu6050_.setYAccelOffset(*ay_o_);
		mpu6050_.setZAccelOffset(*az_o_);

		// Calibrate gyroscope to 0�/s in every axis ()
		if (p_gx_>0) (*gx_o_)--;
		else {(*gx_o_)++;}
		if (p_gy_>0) (*gy_o_)--;
		else {(*gy_o_)++;}
		if (p_gz_>0) (*gz_o_)--;
		else {(*gz_o_)++;}
		
		mpu6050_.setXGyroOffset(*gx_o_);
		mpu6050_.setYGyroOffset(*gy_o_);
		mpu6050_.setZGyroOffset(*gz_o_);
		
		counter_ = 0;
		
		Serial.println("Calibrating GYROSCOPE ACCELEROMETER SENSOR____________________________");
		Serial.print("a[x y z](bits) g[x y z](bits):\t");
		Serial.print(p_ax_); Serial.print("\t");
		Serial.print(p_ay_); Serial.print("\t");
		Serial.print(p_az_); Serial.print("\t");
		Serial.print(p_gx_); Serial.print("\t");
		Serial.print(p_gy_); Serial.print("\t");
		Serial.println(p_gz_);
	}
	counter_++;
}

//unsigned long previousMicros = micros();	// This line is here just in case anyone wants to analyze the processing time

bool GyroscopeAccelerometerManager::update()
{
	bool return_value = false;

	if (command_->commands.gyroacc_calibrate_on)
	{
		if (calibrate_first_run_){
			calibrate_first_run_ = false;
			//readOffsets(); this here reads the offsets from chip, which are not persistant values
		}
		calibrate();

		if (command_->commands.gyroacc_calibrate_off)
		{
			command_->commands.gyroacc_calibrate_on = false;
			command_->commands.gyroacc_calibrate_off = false;
			calibrate_first_run_ = true;
			print_offsets();
		}
	}
	else
	{
		//previousMicros = micros();
		get_readings();
		//Serial.println("Elapsed micros: " + (String)(micros() - previousMicros));		// 2016us elapsed!
		process_readings();
		//Serial.println("Elapsed micros: " + (String)(micros() - previousMicros));		// just 52us elapsed here
		return_value = true;
	}

	if (command_->commands.gyroacc_debug_on)
	{
		print_values();
		if (command_->commands.gyroacc_debug_off)
		{
			command_->commands.gyroacc_debug_on = false;
			command_->commands.gyroacc_debug_off = false;
		}
	}

	return return_value;
}

void GyroscopeAccelerometerManager::print_values()
{
	Serial.println("Reading GYROSCOPE ACCELEROMETER SENSOR____________________________");
	Serial.print("a[x y z](m/s2) g[x y z](deg/s):\t");
	Serial.print(ax_m_s2_); Serial.print("; ");
	Serial.print(ay_m_s2_); Serial.print("; ");
	Serial.print(az_m_s2_); Serial.print("; ");
	Serial.print(gx_deg_s_); Serial.print("; ");
	Serial.print(gy_deg_s_); Serial.print("; ");
	Serial.print(gz_deg_s_); Serial.print("\t");

	// This angle is obtained without complementary filter
// 	Serial.print("X inclination: ");
// 	Serial.print(accel_ang_x_);
// 	Serial.print(" | Y inclination:");
// 	Serial.print(accel_ang_y_); Serial.print("\t");

	// This angle is obtained by complementary filter
	Serial.print("Pitch/Roll angles (x,y):\t");
	Serial.print(get_value_angle_z_pitch_deg());
	Serial.print(" , ");
	Serial.println(get_value_angle_z_roll_deg());
}

void GyroscopeAccelerometerManager::get_acc_values(float& _ax_m_s2, float& _ay_m_s2, float& _az_m_s2)
{
	_ax_m_s2 = ax_m_s2_;
	_ay_m_s2 = ay_m_s2_;
	_az_m_s2 = az_m_s2_;
}

void GyroscopeAccelerometerManager::get_filtered_acc_values(float& _ax_m_s2, float& _ay_m_s2, float& _az_m_s2)
{
	_ax_m_s2 = ax_m_s2_filtered_.get_filtered_value();
	_ay_m_s2 = ay_m_s2_filtered_.get_filtered_value();
	_az_m_s2 = az_m_s2_filtered_.get_filtered_value();
}

void GyroscopeAccelerometerManager::get_gyro_values(float& _gx_deg_s, float& _gy_deg_s, float& _gz_deg_s)
{
	_gx_deg_s = gx_deg_s_;
	_gy_deg_s = gy_deg_s_;
	_gz_deg_s = gz_deg_s_;
}

void GyroscopeAccelerometerManager::get_values(float& _ax_m_s2, float& _ay_m_s2, float& _az_m_s2, float& _gx_deg_s, float& _gy_deg_s, float& _gz_deg_s)
{
	_ax_m_s2 = ax_m_s2_;
	_ay_m_s2 = ay_m_s2_;
	_az_m_s2 = az_m_s2_;
	_gx_deg_s = gx_deg_s_;
	_gy_deg_s = gy_deg_s_;
	_gz_deg_s = gz_deg_s_;
}

void GyroscopeAccelerometerManager::get_value_ax_m_s2(float& _ax_m_s2)
{
	_ax_m_s2 = ax_m_s2_;
}
float GyroscopeAccelerometerManager::get_value_ax_m_s2()
{
	return ax_m_s2_;
}

void GyroscopeAccelerometerManager::get_value_ay_m_s2(float& _ay_m_s2)
{
	_ay_m_s2 = ay_m_s2_;
}
float GyroscopeAccelerometerManager::get_value_ay_m_s2()
{
	return ay_m_s2_;
}

void GyroscopeAccelerometerManager::get_value_az_m_s2(float& _az_m_s2)
{
	_az_m_s2 = az_m_s2_;
}
float GyroscopeAccelerometerManager::get_value_az_m_s2()
{
	return az_m_s2_;
}

void GyroscopeAccelerometerManager::get_value_gx_deg_s(float& _gx_deg_s)
{
	_gx_deg_s = gx_deg_s_;
}
float GyroscopeAccelerometerManager::get_value_gx_deg_s()
{
	return gx_deg_s_;
}

void GyroscopeAccelerometerManager::get_value_gy_deg_s(float& _gy_deg_s)
{
	_gy_deg_s = gy_deg_s_;
}
float GyroscopeAccelerometerManager::get_value_gy_deg_s()
{
	return gy_deg_s_;
}

void GyroscopeAccelerometerManager::get_value_gz_deg_s(float& _gz_deg_s)
{
	_gz_deg_s = gz_deg_s_;
}
float GyroscopeAccelerometerManager::get_value_gz_deg_s()
{
	return gz_deg_s_;
}

void GyroscopeAccelerometerManager::get_value_angle_z_pitch_deg(float& _ang_pitch)
{
	_ang_pitch = rotang_deg_y_;
}

float GyroscopeAccelerometerManager::get_value_angle_z_pitch_deg()
{
	return rotang_deg_y_;
}

void GyroscopeAccelerometerManager::get_value_angle_z_roll_deg(float& _ang_roll)
{
	_ang_roll = - rotang_deg_x_;
}

float GyroscopeAccelerometerManager::get_value_angle_z_roll_deg()
{
	return - rotang_deg_x_;
}

void GyroscopeAccelerometerManager::get_value_angle_z_pitch_rad(float& _ang_pitch)
{
	_ang_pitch = rotang_deg_y_ * DEG_TO_RAD;
}

float GyroscopeAccelerometerManager::get_value_angle_z_pitch_rad()
{
	return rotang_deg_y_ * DEG_TO_RAD;
}

void GyroscopeAccelerometerManager::get_value_angle_z_roll_rad(float& _ang_roll)
{
	_ang_roll = - rotang_deg_x_ * DEG_TO_RAD;
}

float GyroscopeAccelerometerManager::get_value_angle_z_roll_rad()
{
	return - rotang_deg_x_ * DEG_TO_RAD;
}
