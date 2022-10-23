
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
	byte DOs[] = { config->gpio.din_1, config->gpio.din_2 };
	multiple_hx711.configure(DOs, config->gpio.clock);
	
	multiple_hx711.setActiveChannels(false,true,true);
	multiple_hx711.power_up();
	
	filter_ch0_Ax64.setExpConstant(config->filter_exp_constant);
	filter_ch0_Bx32.setExpConstant(config->filter_exp_constant);
	filter_ch1_Ax64.setExpConstant(config->filter_exp_constant);
	filter_ch1_Bx32.setExpConstant(config->filter_exp_constant);
}

void ForceSensorsManager::update()
{
	bool updated = multiple_hx711.update();
	
	if (updated)
	{
		Ax64ChannelValue_0 = filter_ch0_Ax64.filter(static_cast<int32_t>(multiple_hx711.getAx64ChannelValue(0)));
		Bx32ChannelValue_0 = filter_ch0_Bx32.filter(static_cast<int32_t>(multiple_hx711.getBx32ChannelValue(0)));
		Ax64ChannelValue_1 = filter_ch1_Ax64.filter(static_cast<int32_t>(multiple_hx711.getAx64ChannelValue(1)));
		Bx32ChannelValue_1 = filter_ch1_Bx32.filter(static_cast<int32_t>(multiple_hx711.getBx32ChannelValue(1)));
		
		unsigned long timeBetweenReadings = multiple_hx711.getLastElapsedTimeBetweenReadings();
		
		// DEBUG:
		
		Serial.println("Reading ");
		//Serial.print("Ax128: ");
		//Serial.print(Ax128ChannelValue);
		Serial.print("HX711[0] Ax64: \t\t");
		Serial.print(Ax64ChannelValue_0/2);
		Serial.print("\tHX711[0] Bx32: \t\t");
		Serial.println(Bx32ChannelValue_0);
		Serial.print("HX711[1] Ax64: \t\t");
		Serial.print(Ax64ChannelValue_1/2);
		Serial.print("\tHX711[1] Bx32: \t\t");
		Serial.print(Bx32ChannelValue_1);

// 		Serial.print("\treadingTime (us): \t");
// 		Serial.print(elapsedMicros);
 		Serial.print("\tTime between readings (us): \t");
		Serial.print(timeBetweenReadings);
		Serial.println();
		
		if(Serial.available())
		{
			char temp = Serial.read();
			if(temp == 'x')
			{
				Serial.println("TARE");
				//hx711.tare_Ax64(10);
				multiple_hx711.tare_Ax64(0u);
				multiple_hx711.tare_Ax64(1u);
			}
			else if(temp == 'c')
			{
				Serial.println("TARE");
				//hx711.tare_Bx32(10);
				multiple_hx711.tare_Bx32(0u);
				multiple_hx711.tare_Bx32(1u);
			}
		}
		
		// END DEBUG

	}
	
}

long ForceSensorsManager::getValue_LeftFoot_LeftFrontSensor()
{
	//TODO: Check which channel
	return Ax64ChannelValue_0;
}
long ForceSensorsManager::getValue_LeftFoot_RightFrontSensor()
{
	//TODO: Check which channel
	return Bx32ChannelValue_0;
}
long ForceSensorsManager::getValue_LeftFoot_LeftBackSensor()
{
	//TODO: Check which channel
	return Ax64ChannelValue_1;
}
long ForceSensorsManager::getValue_LeftFoot_RightBackSensor()
{
	//TODO: Check which channel
	return Bx32ChannelValue_1;
}

//TODO: fill
long ForceSensorsManager::getValue_RightFoot_LeftFrontSensor(){}
long ForceSensorsManager::getValue_RightFoot_RightFrontSensor(){}
long ForceSensorsManager::getValue_RightFoot_LeftBackSensor(){}
long ForceSensorsManager::getValue_RightFoot_RightBackSensor(){}
	