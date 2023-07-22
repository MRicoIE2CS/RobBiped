/*
 * ForceSensorsManager.cpp
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

#include "ForceSensorsManager.h"

void ForceSensorsManager::assoc_config(Configs::ForceSensors &_config){
	config_ = &_config;
}

void ForceSensorsManager::init()
{	
	// Get Command singleton instance
	command_ = Command::get_instance();

	// Important! Setting active channels needs to be called before configure() method because of persistent storage issues and performance
	multiple_hx711_.set_active_channels(config_->active_channels._Ax128, config_->active_channels._Ax64, config_->active_channels._Bx32);
	multiple_hx711_.configure(config_->gpio.dINs, config_->gpio.clock);

	calibration_LeftFoot_LeftFront_cell_ = &(config_->calibration_LeftFoot_LeftFront_cell);
	calibration_LeftFoot_RightFront_cell_ = &(config_->calibration_LeftFoot_RightFront_cell);
	calibration_LeftFoot_LeftBack_cell_ = &(config_->calibration_LeftFoot_LeftBack_cell);
	calibration_LeftFoot_RightBack_cell_ = &(config_->calibration_LeftFoot_RightBack_cell);
	calibration_RightFoot_LeftFront_cell_ = &(config_->calibration_RightFoot_LeftFront_cell);
	calibration_RightFoot_RightFront_cell_ = &(config_->calibration_RightFoot_RightFront_cell);
	calibration_RightFoot_LeftBack_cell_ = &(config_->calibration_RightFoot_LeftBack_cell);
	calibration_RightFoot_RightBack_cell_ = &(config_->calibration_RightFoot_RightBack_cell);

	separation_FrontBack_mm_ = &(config_->location_mm.frontBack_separation);
	separation_LeftRight_mm_ = &(config_->location_mm.leftRight_separation);

	filter_LeftFoot_LeftBack_.set_time_constant(config_->filter_time_constant_ms);
	filter_LeftFoot_LeftFront_.set_time_constant(config_->filter_time_constant_ms);
	filter_LeftFoot_RightBack_.set_time_constant(config_->filter_time_constant_ms);
	filter_LeftFoot_RightFront_.set_time_constant(config_->filter_time_constant_ms);
	filter_RightFoot_LeftBack_.set_time_constant(config_->filter_time_constant_ms);
	filter_RightFoot_LeftFront_.set_time_constant(config_->filter_time_constant_ms);
	filter_RightFoot_RightBack_.set_time_constant(config_->filter_time_constant_ms);
	filter_RightFoot_RightFront_.set_time_constant(config_->filter_time_constant_ms);

	filter_LeftFoot_LeftBack_.set_threshold_value(config_->filter_threshold_value);
	filter_LeftFoot_LeftFront_.set_threshold_value(config_->filter_threshold_value);
	filter_LeftFoot_RightBack_.set_threshold_value(config_->filter_threshold_value);
	filter_LeftFoot_RightFront_.set_threshold_value(config_->filter_threshold_value);
	filter_RightFoot_LeftBack_.set_threshold_value(config_->filter_threshold_value);
	filter_RightFoot_LeftFront_.set_threshold_value(config_->filter_threshold_value);
	filter_RightFoot_RightBack_.set_threshold_value(config_->filter_threshold_value);
	filter_RightFoot_RightFront_.set_threshold_value(config_->filter_threshold_value);

	//______//

	multiple_hx711_.power_up();
}

/*
// VIRTUAL CONNEXIONS TABLE
//

LeftFoot_LeftBack_		->		Ax64 of HX711(0u)
LeftFoot_LeftFront_		->		Ax64 of HX711(1u)
LeftFoot_RightBack_		->		Ax64 of HX711(2u)
LeftFoot_RightFront_	->		Ax64 of HX711(3u)
RightFoot_LeftBack_		->		Ax64 of HX711(4u)
RightFoot_LeftFront_	->		Ax64 of HX711(5u)
RightFoot_RightBack_	->		Ax64 of HX711(6u)
RightFoot_RightFront_	->		Ax64 of HX711(7u)

*/ 

bool ForceSensorsManager::update()
{
	bool updated = multiple_hx711_.update();

	if (updated)
	{
 		value_LeftFoot_LeftBack_ = filter_LeftFoot_LeftBack_.filter_pr(multiple_hx711_.get_Ax64_channel_value(0u)) * *calibration_LeftFoot_LeftBack_cell_;
		value_LeftFoot_LeftFront_ = filter_LeftFoot_LeftFront_.filter_pr(multiple_hx711_.get_Ax64_channel_value(1u)) * *calibration_LeftFoot_LeftFront_cell_;
		value_LeftFoot_RightBack_ = filter_LeftFoot_RightBack_.filter_pr(multiple_hx711_.get_Ax64_channel_value(2u)) * *calibration_LeftFoot_RightBack_cell_;
		value_LeftFoot_RightFront_ = filter_LeftFoot_RightFront_.filter_pr(multiple_hx711_.get_Ax64_channel_value(3u)) * *calibration_LeftFoot_RightFront_cell_;
		value_RightFoot_LeftBack_ = filter_RightFoot_LeftBack_.filter_pr(multiple_hx711_.get_Ax64_channel_value(4u)) * *calibration_RightFoot_LeftBack_cell_;
		value_RightFoot_LeftFront_ = filter_RightFoot_LeftFront_.filter_pr(multiple_hx711_.get_Ax64_channel_value(5u)) * *calibration_RightFoot_LeftFront_cell_;
		value_RightFoot_RightBack_ = filter_RightFoot_RightBack_.filter_pr(multiple_hx711_.get_Ax64_channel_value(6u)) * *calibration_RightFoot_RightBack_cell_;
		value_RightFoot_RightFront_ = filter_RightFoot_RightFront_.filter_pr(multiple_hx711_.get_Ax64_channel_value(7u)) * *calibration_RightFoot_RightFront_cell_;
		
		calculate_ZMP();
		
		if (command_->commands.force_debug_on)
		{
			print_values();
			if (command_->commands.force_debug_off)
			{
				command_->commands.force_debug_on = false;
				command_->commands.force_debug_off = false;
			}
		}
		
		if (command_->commands.zmp_debug_on)
		{
			print_ZMP();
			if (command_->commands.zmp_debug_off)
			{
				command_->commands.zmp_debug_on = false;
				command_->commands.zmp_debug_off = false;
			}
		}
	}

	if (command_->commands.force_tare_left)
	{
		tare_LeftFoot();
		
		command_->commands.force_tare_left = false;
	}
	if (command_->commands.force_tare_right)
	{
		tare_RightFoot();
		
		command_->commands.force_tare_right = false;
	}

	return updated;
}

int32_t ForceSensorsManager::getValue_gr_LeftFoot_LeftBackSensor()
{
	return value_LeftFoot_LeftBack_;
}
int32_t ForceSensorsManager::getValue_gr_LeftFoot_LeftFrontSensor()
{
	return value_LeftFoot_LeftFront_;
}
int32_t ForceSensorsManager::getValue_gr_LeftFoot_RightBackSensor()
{
	return value_LeftFoot_RightBack_;
}
int32_t ForceSensorsManager::getValue_gr_LeftFoot_RightFrontSensor()
{
	return value_LeftFoot_RightFront_;
}

int32_t ForceSensorsManager::getValue_gr_RightFoot_LeftBackSensor()
{
	return value_RightFoot_LeftBack_;
}

int32_t ForceSensorsManager::getValue_gr_RightFoot_LeftFrontSensor()
{
	return value_RightFoot_LeftFront_;
}

int32_t ForceSensorsManager::getValue_gr_RightFoot_RightBackSensor()
{
	return value_RightFoot_RightBack_;
}

int32_t ForceSensorsManager::getValue_gr_RightFoot_RightFrontSensor()
{
	return value_RightFoot_RightFront_;
}

uint32_t ForceSensorsManager::get_last_elapsed_time_between_readings()
{
	return multiple_hx711_.get_last_elapsed_time_between_readings();
}

void ForceSensorsManager::tare_LeftFoot()
{
	multiple_hx711_.tare_Ax64(0u);
	filter_LeftFoot_LeftBack_.filter_pr(multiple_hx711_.get_Ax64_channel_value(0u), true);	// This is done for the peak reject filter to accept this calibration step
	multiple_hx711_.tare_Ax64(1u);
	filter_LeftFoot_LeftFront_.filter_pr(multiple_hx711_.get_Ax64_channel_value(1u), true);
	multiple_hx711_.tare_Ax64(2u);
	filter_LeftFoot_RightBack_.filter_pr(multiple_hx711_.get_Ax64_channel_value(2u), true);
	multiple_hx711_.tare_Ax64(3u);
	filter_LeftFoot_RightFront_.filter_pr(multiple_hx711_.get_Ax64_channel_value(3u), true);

	is_tare_left_performed_ = true;
}

void ForceSensorsManager::tare_RightFoot()
{
	multiple_hx711_.tare_Ax64(4u);
	filter_RightFoot_LeftBack_.filter_pr(multiple_hx711_.get_Ax64_channel_value(4u), true);
	multiple_hx711_.tare_Ax64(5u);
	filter_RightFoot_LeftFront_.filter_pr(multiple_hx711_.get_Ax64_channel_value(5u), true);
	multiple_hx711_.tare_Ax64(6u);
	filter_RightFoot_RightBack_.filter_pr(multiple_hx711_.get_Ax64_channel_value(6u), true);
	multiple_hx711_.tare_Ax64(7u);
	filter_RightFoot_RightFront_.filter_pr(multiple_hx711_.get_Ax64_channel_value(7u), true);

	is_tare_right_performed_ = true;
}

void ForceSensorsManager::calculate_ZMP()
{
	int32_t force_sum =
			value_LeftFoot_LeftBack_ + value_LeftFoot_LeftFront_ + value_LeftFoot_RightBack_ + value_LeftFoot_RightFront_;
	int32_t force_ponderatedSum =
			(value_LeftFoot_LeftFront_ * *separation_FrontBack_mm_) + (value_LeftFoot_RightFront_ * *separation_FrontBack_mm_);

	// ZMP X coordinate of the left foot, in mm, from the left-back sensor
	zmp_left_foot_x_mm_ = filter_zmp_left_foot_x_mm_.filter((force_sum == 0) ? 0 : force_ponderatedSum / force_sum);
	// ZMP coordinate frame translation to the foot center
	zmp_left_foot_x_mm_ -= *separation_FrontBack_mm_ / 2;

	// Y coordinate axis is positive pointing to the outside of the body
	force_ponderatedSum =
			(value_LeftFoot_LeftBack_ * *separation_LeftRight_mm_) + (value_LeftFoot_LeftFront_ * *separation_LeftRight_mm_);

	// ZMP Y coordinate of the left foot, in mm, from the left-back sensor
	zmp_left_foot_y_mm_ = filter_zmp_left_foot_y_mm_.filter((force_sum == 0) ? 0 : force_ponderatedSum / force_sum);
	// ZMP coordinate frame translation to the foot center
	zmp_left_foot_y_mm_ -= *separation_LeftRight_mm_ / 2;

	force_sum =
			value_RightFoot_LeftBack_ + value_RightFoot_LeftFront_ + value_RightFoot_RightBack_ + value_RightFoot_RightFront_;
	force_ponderatedSum =
			(value_RightFoot_LeftFront_ * *separation_FrontBack_mm_) + (value_RightFoot_RightFront_ * *separation_FrontBack_mm_);

	// ZMP X coordinate of the right foot, in mm, from the left-back sensor
	zmp_right_foot_x_mm_ = filter_zmp_right_foot_x_mm_.filter((force_sum == 0) ? 0 : force_ponderatedSum / force_sum);
	// ZMP coordinate frame translation to the foot center
	zmp_right_foot_x_mm_ -= *separation_FrontBack_mm_ / 2;

	// Y coordinate axis is positive pointing to the outside of the body
	force_ponderatedSum =
			(value_RightFoot_RightBack_ * *separation_LeftRight_mm_) + (value_RightFoot_RightFront_ * *separation_LeftRight_mm_);

	// ZMP Y coordinate of the right foot, in mm, from the left-back sensor
	zmp_right_foot_y_mm_ = filter_zmp_right_foot_y_mm_.filter((force_sum == 0) ? 0 : force_ponderatedSum / force_sum);
	// ZMP coordinate frame translation to the foot center
	zmp_right_foot_y_mm_ -= *separation_LeftRight_mm_ / 2;
}

void ForceSensorsManager::get_values_ZMP_LeftFoot(int16_t& x_mm, int16_t& y_mm)
{
	x_mm = zmp_left_foot_x_mm_;
	y_mm = zmp_left_foot_y_mm_;
}

void ForceSensorsManager::get_values_ZMP_RightFoot(int16_t& x_mm, int16_t& y_mm)
{
	x_mm = zmp_right_foot_x_mm_;
	y_mm = zmp_right_foot_y_mm_;
}

void ForceSensorsManager::print_values()
{
	Serial.println("Reading FORCE SENSORS____________________________");
	Serial.print("LeftFoot: \t\t");
	Serial.print(getValue_gr_LeftFoot_LeftFrontSensor());
	Serial.print("\t");
	Serial.println(getValue_gr_LeftFoot_RightFrontSensor());
	Serial.print("\t\t\t");
	Serial.print(getValue_gr_LeftFoot_LeftBackSensor());
	Serial.print("\t");
	Serial.println(getValue_gr_LeftFoot_RightBackSensor());


	Serial.print("RightFoot: \t\t");
	Serial.print(getValue_gr_RightFoot_LeftFrontSensor());
	Serial.print("\t");
	Serial.println(getValue_gr_RightFoot_RightFrontSensor());
	Serial.print("\t\t\t");
	Serial.print(getValue_gr_RightFoot_LeftBackSensor());
	Serial.print("\t");
	Serial.println(getValue_gr_RightFoot_RightBackSensor());

//  	Serial.println("\tTime between readings (us): \t");
//  	Serial.println(get_last_elapsed_time_between_readings());
}

void ForceSensorsManager::print_ZMP()
{
	Serial.println("Reading ZMP coordinates____________________________");
	Serial.print("Left foot (X,Y):\t\t");
	Serial.print(zmp_left_foot_x_mm_);
	Serial.print("\t");
	Serial.println(zmp_left_foot_y_mm_);
	Serial.print("Right foot (X,Y):\t\t");
	Serial.print(zmp_right_foot_x_mm_);
	Serial.print("\t");
	Serial.println(zmp_right_foot_y_mm_);
}

bool ForceSensorsManager::is_tare_left_performed()
{
	return is_tare_left_performed_;
}

bool ForceSensorsManager::is_tare_right_performed()
{
	return is_tare_right_performed_;
}
