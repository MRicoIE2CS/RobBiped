/*
 * ForceSensorsManager.cpp
 *
 * Copyright 2023 Mikel Rico Abajo (MRicoIE2C)

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

void ForceSensorsManager::assocConfig(Configs::ForceSensors &_config){
	config = &_config;
}

void ForceSensorsManager::init()
{	
	// Get Command singleton instance
	command = Command::getInstance();

	multiple_hx711.configure(config->gpio.dINs, config->gpio.clock);
	
	calibration_LeftFoot_LeftFrontSensor = &(config->calibration_LeftFoot_LeftFrontSensor);
	calibration_LeftFoot_RightFrontSensor = &(config->calibration_LeftFoot_RightFrontSensor);
	calibration_LeftFoot_LeftBackSensor = &(config->calibration_LeftFoot_LeftBackSensor);
	calibration_LeftFoot_RightBackSensor = &(config->calibration_LeftFoot_RightBackSensor);
	calibration_RightFoot_LeftFrontSensor = &(config->calibration_RightFoot_LeftFrontSensor);
	calibration_RightFoot_RightFrontSensor = &(config->calibration_RightFoot_RightFrontSensor);
	calibration_RightFoot_LeftBackSensor = &(config->calibration_RightFoot_LeftBackSensor);
	calibration_RightFoot_RightBackSensor = &(config->calibration_RightFoot_RightBackSensor);
	
	separation_frontBack_mm = &(config->location_mm.frontBack_separation);
	separation_leftRight_mm = &(config->location_mm.leftRight_separation);
	
	multiple_hx711.setActiveChannels(false,true,true);
	multiple_hx711.power_up();
	
	//______//
	
	filter_LeftFoot_LeftBack.setExpConstant(config->filter_exp_constant);
	filter_LeftFoot_LeftFront.setExpConstant(config->filter_exp_constant);
	filter_LeftFoot_RightBack.setExpConstant(config->filter_exp_constant);
	filter_LeftFoot_RightFront.setExpConstant(config->filter_exp_constant);
	filter_RightFoot_LeftBack.setExpConstant(config->filter_exp_constant);
	filter_RightFoot_LeftFront.setExpConstant(config->filter_exp_constant);
	filter_RightFoot_RightBack.setExpConstant(config->filter_exp_constant);
	filter_RightFoot_RightFront.setExpConstant(config->filter_exp_constant);
	
	filter_LeftFoot_LeftBack.setThresholdValue(config->filter_threshold_value);
	filter_LeftFoot_LeftFront.setThresholdValue(config->filter_threshold_value);
	filter_LeftFoot_RightBack.setThresholdValue(config->filter_threshold_value);
	filter_LeftFoot_RightFront.setThresholdValue(config->filter_threshold_value);
	filter_RightFoot_LeftBack.setThresholdValue(config->filter_threshold_value);
	filter_RightFoot_LeftFront.setThresholdValue(config->filter_threshold_value);
	filter_RightFoot_RightBack.setThresholdValue(config->filter_threshold_value);
	filter_RightFoot_RightFront.setThresholdValue(config->filter_threshold_value);
}

bool ForceSensorsManager::update()
{
	bool updated = multiple_hx711.update();

	if (updated)
	{
		value_LeftFoot_LeftBack = filter_LeftFoot_LeftBack.filter_pr(multiple_hx711.getAx64ChannelValue(0u)) / 2.0 * *calibration_LeftFoot_LeftBackSensor;
		value_LeftFoot_LeftFront = filter_LeftFoot_LeftFront.filter_pr(multiple_hx711.getBx32ChannelValue(0u)) * *calibration_LeftFoot_LeftFrontSensor;
		value_LeftFoot_RightBack = filter_LeftFoot_RightBack.filter_pr(multiple_hx711.getAx64ChannelValue(1u)) / 2.0 * *calibration_LeftFoot_RightBackSensor;
		value_LeftFoot_RightFront = filter_LeftFoot_RightFront.filter_pr(multiple_hx711.getBx32ChannelValue(1u)) * *calibration_LeftFoot_RightFrontSensor;
		value_RightFoot_LeftBack = filter_RightFoot_LeftBack.filter_pr(multiple_hx711.getAx64ChannelValue(2u)) / 2.0 * *calibration_RightFoot_LeftBackSensor;
		value_RightFoot_LeftFront = filter_RightFoot_LeftFront.filter_pr(multiple_hx711.getBx32ChannelValue(2u)) * *calibration_RightFoot_LeftFrontSensor;
		value_RightFoot_RightBack = filter_RightFoot_RightBack.filter_pr(multiple_hx711.getAx64ChannelValue(3u)) / 2.0 * *calibration_RightFoot_RightBackSensor;
		value_RightFoot_RightFront = filter_RightFoot_RightFront.filter_pr(multiple_hx711.getBx32ChannelValue(3u)) * *calibration_RightFoot_RightFrontSensor;
		
		calculate_ZMP();
	}

	if (command->commands.force_tare_left)
	{
		Serial.println("TARE");
		tare_LeftFoot();
		
		command->commands.force_tare_left = false;
	}
	if (command->commands.force_tare_right)
	{
		Serial.println("TARE");
		tare_RightFoot();
		
		command->commands.force_tare_right = false;
	}
	
	if (command->commands.force_debug_on)
	{
		printValues();
		if (command->commands.force_debug_off)
		{
			command->commands.force_debug_on = false;
			command->commands.force_debug_off = false;
		}
	}
	
	if (command->commands.zmp_debug_on)
	{
		printZMP();
		if (command->commands.zmp_debug_off)
		{
			command->commands.zmp_debug_on = false;
			command->commands.zmp_debug_off = false;
		}
	}

	return updated;
}

int32_t ForceSensorsManager::getValue_gr_LeftFoot_LeftBackSensor()
{
	return value_LeftFoot_LeftBack;
}
int32_t ForceSensorsManager::getValue_gr_LeftFoot_LeftFrontSensor()
{
	return value_LeftFoot_LeftFront;
}
int32_t ForceSensorsManager::getValue_gr_LeftFoot_RightBackSensor()
{
	return value_LeftFoot_RightBack;
}
int32_t ForceSensorsManager::getValue_gr_LeftFoot_RightFrontSensor()
{
	return value_LeftFoot_RightFront;
}

int32_t ForceSensorsManager::getValue_gr_RightFoot_LeftBackSensor()
{
	return value_RightFoot_LeftBack;
}

int32_t ForceSensorsManager::getValue_gr_RightFoot_LeftFrontSensor()
{
	return value_RightFoot_LeftFront;
}

int32_t ForceSensorsManager::getValue_gr_RightFoot_RightBackSensor()
{
	return value_RightFoot_RightBack;
}

int32_t ForceSensorsManager::getValue_gr_RightFoot_RightFrontSensor()
{
	return value_RightFoot_RightFront;
}

uint32_t ForceSensorsManager::getLastElapsedTimeBetweenReadings()
{
	return multiple_hx711.getLastElapsedTimeBetweenReadings();
}

void ForceSensorsManager::tare_LeftFoot()
{
	multiple_hx711.tare_Ax64(0u);
	filter_LeftFoot_LeftBack.filter_pr(multiple_hx711.getAx64ChannelValue(0u), true);	// This is done for the peak reject filter to accept this calibration step
	multiple_hx711.tare_Bx32(0u);
	filter_LeftFoot_LeftFront.filter_pr(multiple_hx711.getBx32ChannelValue(0u), true);
	multiple_hx711.tare_Ax64(1u);
	filter_LeftFoot_RightBack.filter_pr(multiple_hx711.getAx64ChannelValue(1u), true);
	multiple_hx711.tare_Bx32(1u);
	filter_LeftFoot_RightFront.filter_pr(multiple_hx711.getBx32ChannelValue(1u), true);
}

void ForceSensorsManager::tare_RightFoot()
{
	multiple_hx711.tare_Ax64(2u);
	filter_RightFoot_LeftBack.filter_pr(multiple_hx711.getAx64ChannelValue(2u), true);
	multiple_hx711.tare_Bx32(2u);
	filter_RightFoot_LeftFront.filter_pr(multiple_hx711.getBx32ChannelValue(2u), true);
	multiple_hx711.tare_Ax64(3u);
	filter_RightFoot_RightBack.filter_pr(multiple_hx711.getAx64ChannelValue(3u), true);
	multiple_hx711.tare_Bx32(3u);
	filter_RightFoot_RightFront.filter_pr(multiple_hx711.getBx32ChannelValue(3u), true);
}

void ForceSensorsManager::calculate_ZMP()
{
	int32_t force_sum =
			value_LeftFoot_LeftBack + value_LeftFoot_LeftFront + value_LeftFoot_RightBack + value_LeftFoot_RightFront;
	int32_t force_ponderatedSum =
			value_LeftFoot_LeftBack + (value_LeftFoot_LeftFront * *separation_frontBack_mm) + value_LeftFoot_RightBack + (value_LeftFoot_RightFront * *separation_frontBack_mm);
	
	// ZMP X coordinate of the left foot, in mm, from the left-back sensor
	zmp_left_foot_x_mm = filter_zmp_left_foot_x_mm.filter((force_sum == 0) ? 0 : force_ponderatedSum / force_sum);
	
	force_ponderatedSum =
			value_LeftFoot_LeftBack + value_LeftFoot_LeftFront + (value_LeftFoot_RightBack * *separation_leftRight_mm) + (value_LeftFoot_RightFront * *separation_leftRight_mm);
	
	// ZMP Y coordinate of the left foot, in mm, from the left-back sensor
	zmp_left_foot_y_mm = filter_zmp_left_foot_y_mm.filter((force_sum == 0) ? 0 : force_ponderatedSum / force_sum);
	
	force_sum =
			value_RightFoot_LeftBack + value_RightFoot_LeftFront + value_RightFoot_RightBack + value_RightFoot_RightFront;
	force_ponderatedSum =
			value_RightFoot_LeftBack + (value_RightFoot_LeftFront * *separation_frontBack_mm) + value_RightFoot_RightBack + (value_RightFoot_RightFront * *separation_frontBack_mm);
	
	// ZMP X coordinate of the right foot, in mm, from the left-back sensor
	zmp_right_foot_x_mm = filter_zmp_right_foot_x_mm.filter((force_sum == 0) ? 0 : force_ponderatedSum / force_sum);
	
	force_ponderatedSum =
			value_RightFoot_LeftBack + value_RightFoot_LeftFront + (value_RightFoot_RightBack * *separation_leftRight_mm) + (value_RightFoot_RightFront * *separation_leftRight_mm);
	
	// ZMP Y coordinate of the right foot, in mm, from the left-back sensor
	zmp_right_foot_y_mm = filter_zmp_right_foot_y_mm.filter((force_sum == 0) ? 0 : force_ponderatedSum / force_sum);
}

void ForceSensorsManager::getValues_ZMP_leftFoot(int16_t& x_mm, int16_t& y_mm)
{
	x_mm = zmp_left_foot_x_mm;
	y_mm = zmp_left_foot_y_mm;
}

void ForceSensorsManager::getValues_ZMP_rightFoot(int16_t& x_mm, int16_t& y_mm)
{
	x_mm = zmp_right_foot_x_mm;
	y_mm = zmp_right_foot_y_mm;
}

void ForceSensorsManager::printValues()
{
	Serial.println("Reading FORCE SENSORS____________________________");
	Serial.print("LeftFoot_LeftFrontSensor: \t\t");
	Serial.print(getValue_gr_LeftFoot_LeftFrontSensor());
	Serial.print("\tLeftFoot_RightFrontSensor: \t\t");
	Serial.println(getValue_gr_LeftFoot_RightFrontSensor());
	Serial.print("LeftFoot_LeftBackSensor: \t\t");
	Serial.print(getValue_gr_LeftFoot_LeftBackSensor());
	Serial.print("\tLeftFoot_RightBackSensor: \t\t");
	Serial.println(getValue_gr_LeftFoot_RightBackSensor());


	Serial.print("RightFoot_LeftFrontSensor: \t\t");
	Serial.print(getValue_gr_RightFoot_LeftFrontSensor());
	Serial.print("\tRightFoot_RightFrontSensor: \t\t");
	Serial.println(getValue_gr_RightFoot_RightFrontSensor());
	Serial.print("RightFoot_LeftBackSensor: \t\t");
	Serial.print(getValue_gr_RightFoot_LeftBackSensor());
	Serial.print("\tRightFoot_RightBackSensor: \t\t");
	Serial.println(getValue_gr_RightFoot_RightBackSensor());

// 	Serial.println("\tTime between readings (us): \t");
// 	Serial.println(getLastElapsedTimeBetweenReadings());
}

void ForceSensorsManager::printZMP()
{
	Serial.println("Reading ZMP coordinates____________________________");
	Serial.print("Left foot X mm(): \t\t");
	Serial.println(zmp_left_foot_x_mm);
	Serial.print("Left foot Y mm(): \t\t");
	Serial.println(zmp_left_foot_y_mm);
	Serial.print("Right foot X mm(): \t\t");
	Serial.println(zmp_right_foot_x_mm);
	Serial.print("Right foot X mm(): \t\t");
	Serial.println(zmp_right_foot_y_mm);
}
