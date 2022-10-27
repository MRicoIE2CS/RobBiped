
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
	
	multiple_hx711.setActiveChannels(config->activeChannels.Ax128,config->activeChannels.Ax64,config->activeChannels.Bx32);
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
		Ax64ChannelValue_0 = filter_ch0_Ax64.filter(static_cast<int32_t>(multiple_hx711.getAx64ChannelValue(0)));
		Bx32ChannelValue_0 = filter_ch0_Bx32.filter(static_cast<int32_t>(multiple_hx711.getBx32ChannelValue(0)));
		Ax64ChannelValue_1 = filter_ch1_Ax64.filter(static_cast<int32_t>(multiple_hx711.getAx64ChannelValue(1)));
		Bx32ChannelValue_1 = filter_ch1_Bx32.filter(static_cast<int32_t>(multiple_hx711.getBx32ChannelValue(1)));
		Ax64ChannelValue_2 = filter_ch2_Ax64.filter(static_cast<int32_t>(multiple_hx711.getAx64ChannelValue(2)));
		Bx32ChannelValue_2 = filter_ch2_Bx32.filter(static_cast<int32_t>(multiple_hx711.getBx32ChannelValue(2)));
		Ax64ChannelValue_3 = filter_ch3_Ax64.filter(static_cast<int32_t>(multiple_hx711.getAx64ChannelValue(3)));
		Bx32ChannelValue_3 = filter_ch3_Bx32.filter(static_cast<int32_t>(multiple_hx711.getBx32ChannelValue(3)));
	}
	
	return updated;
}

long ForceSensorsManager::getValue_LeftFoot_LeftFrontSensor()
{
	return Ax64ChannelValue_0/2;
}
long ForceSensorsManager::getValue_LeftFoot_RightFrontSensor()
{
	return Bx32ChannelValue_0;
}
long ForceSensorsManager::getValue_LeftFoot_LeftBackSensor()
{
	return Ax64ChannelValue_1/2;
}
long ForceSensorsManager::getValue_LeftFoot_RightBackSensor()
{
	return Bx32ChannelValue_1;
}

long ForceSensorsManager::getValue_RightFoot_LeftFrontSensor()
{
	return Ax64ChannelValue_2/2;
}

long ForceSensorsManager::getValue_RightFoot_RightFrontSensor()
{
	return Bx32ChannelValue_2;
}

long ForceSensorsManager::getValue_RightFoot_LeftBackSensor()
{
	return Ax64ChannelValue_3/2;
}

long ForceSensorsManager::getValue_RightFoot_RightBackSensor()
{
	return Bx32ChannelValue_3;
}

unsigned long ForceSensorsManager::getLastElapsedTimeBetweenReadings()
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
