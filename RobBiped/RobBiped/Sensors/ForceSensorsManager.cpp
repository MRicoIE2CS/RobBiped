
/*
 * ForceSensorsManager.cpp
 *
 * Created: 20/10/2022
 * Author: MRICO
 */ 

#include "ForceSensorsManager.h"

void ForceSensorsManager::assocConfig(Configs::ForceSensors &_config){
	config = &_config;
}

void ForceSensorsManager::init()
{
	multiple_hx711.configure(config->gpio.dINs, config->gpio.clock);
	
	calibration_LeftFoot_LeftFrontSensor = &(config->calibration_LeftFoot_LeftFrontSensor);
	calibration_LeftFoot_RightFrontSensor = &(config->calibration_LeftFoot_RightFrontSensor);
	calibration_LeftFoot_LeftBackSensor = &(config->calibration_LeftFoot_LeftBackSensor);
	calibration_LeftFoot_RightBackSensor = &(config->calibration_LeftFoot_RightBackSensor);
	calibration_RightFoot_LeftFrontSensor = &(config->calibration_RightFoot_LeftFrontSensor);
	calibration_RightFoot_RightFrontSensor = &(config->calibration_RightFoot_RightFrontSensor);
	calibration_RightFoot_LeftBackSensor = &(config->calibration_RightFoot_LeftBackSensor);
	calibration_RightFoot_RightBackSensor = &(config->calibration_RightFoot_RightBackSensor);
	
	multiple_hx711.setActiveChannels(false,true,true);
	multiple_hx711.power_up();
	
	//______//
	
	filter_ch0_Ax64.setExpConstant(config->filter_exp_constant);
	filter_ch0_Bx32.setExpConstant(config->filter_exp_constant);
	filter_ch1_Ax64.setExpConstant(config->filter_exp_constant);
	filter_ch1_Bx32.setExpConstant(config->filter_exp_constant);
	filter_ch2_Ax64.setExpConstant(config->filter_exp_constant);
	filter_ch2_Bx32.setExpConstant(config->filter_exp_constant);
	filter_ch3_Ax64.setExpConstant(config->filter_exp_constant);
	filter_ch3_Bx32.setExpConstant(config->filter_exp_constant);
}

bool ForceSensorsManager::update()
{
	bool updated = multiple_hx711.update();
	
	if (updated)
	{
		// Readings obtention
		Ax64ChannelValue_0 = filter_ch0_Ax64.filter(multiple_hx711.getAx64ChannelValue(0u));
		Bx32ChannelValue_0 = filter_ch0_Bx32.filter(multiple_hx711.getBx32ChannelValue(0u));
		Ax64ChannelValue_1 = filter_ch1_Ax64.filter(multiple_hx711.getAx64ChannelValue(1u));
		Bx32ChannelValue_1 = filter_ch1_Bx32.filter(multiple_hx711.getBx32ChannelValue(1u));
		Ax64ChannelValue_2 = filter_ch2_Ax64.filter(multiple_hx711.getAx64ChannelValue(2u));
		Bx32ChannelValue_2 = filter_ch2_Bx32.filter(multiple_hx711.getBx32ChannelValue(2u));
		Ax64ChannelValue_3 = filter_ch3_Ax64.filter(multiple_hx711.getAx64ChannelValue(3u));
		Bx32ChannelValue_3 = filter_ch3_Bx32.filter(multiple_hx711.getBx32ChannelValue(3u));
	}
	
	return updated;
}

int32_t ForceSensorsManager::getValue_LeftFoot_LeftBackSensor()
{
	return Ax64ChannelValue_0 / 2.0 * *calibration_LeftFoot_LeftBackSensor;
}
int32_t ForceSensorsManager::getValue_LeftFoot_LeftFrontSensor()
{
	return Bx32ChannelValue_0 * *calibration_LeftFoot_LeftFrontSensor;
}
int32_t ForceSensorsManager::getValue_LeftFoot_RightBackSensor()
{
	return Ax64ChannelValue_1 / 2.0 * *calibration_LeftFoot_RightBackSensor;
}
int32_t ForceSensorsManager::getValue_LeftFoot_RightFrontSensor()
{
	return Bx32ChannelValue_1 * *calibration_LeftFoot_RightFrontSensor;
}

int32_t ForceSensorsManager::getValue_RightFoot_LeftBackSensor()
{
	return Ax64ChannelValue_2 / 2.0 * *calibration_RightFoot_LeftBackSensor;
}

int32_t ForceSensorsManager::getValue_RightFoot_LeftFrontSensor()
{
	return Bx32ChannelValue_2 * *calibration_RightFoot_LeftFrontSensor;
}

int32_t ForceSensorsManager::getValue_RightFoot_RightBackSensor()
{
	return Ax64ChannelValue_3 / 2.0 * *calibration_RightFoot_RightBackSensor;
}

int32_t ForceSensorsManager::getValue_RightFoot_RightFrontSensor()
{
	return Bx32ChannelValue_3 * *calibration_RightFoot_RightFrontSensor;
}

uint32_t ForceSensorsManager::getLastElapsedTimeBetweenReadings()
{
	return multiple_hx711.getLastElapsedTimeBetweenReadings();
}

void ForceSensorsManager::tare_LeftFoot()
{
	multiple_hx711.tare_Ax64(0u);
	multiple_hx711.tare_Bx32(0u);
	multiple_hx711.tare_Ax64(1u);
	multiple_hx711.tare_Bx32(1u);
}

void ForceSensorsManager::tare_RightFoot()
{
	multiple_hx711.tare_Ax64(2u);
	multiple_hx711.tare_Bx32(2u);
	multiple_hx711.tare_Ax64(3u);
	multiple_hx711.tare_Bx32(3u);
}
