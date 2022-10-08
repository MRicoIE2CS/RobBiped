

/*
 * RobBiped.ino
 *
 * Created: 1/28/2022 8:02:12 PM
 * Author: MRICO
 */ 

#include "Main/Executer.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <vector>

#include "Sensors/HX711/HX711.h"
#include "Sensors/HX711/multiple_HX711.h"

#define SCKpin 18
#define DOpin1 19
#define DOpin2 5
byte DOs[] = { DOpin1, DOpin2 };
Multiple_HX711 multiple_hx711(DOs, SCKpin);
//HX711 hx711(DOpin1, SCKpin, 1);

float calibration_factor = 1;	// Erasable: Used for fine calibration over debug prints

Executer executer;

void setup()
{
	Serial.begin(500000);
	
	//executer.init();
	//while(!(Serial.available())){};
	
	Serial.println("Initial delay...");
	delay(3000);
	Serial.println("3");
	delay(1000);
	Serial.println("2");
	delay(1000);
	Serial.println("1");
	delay(1000);

	//hx711.set_scale(calibration_factor);
// 	hx711.setActiveChannels(false,true,true);
// 	hx711.power_up();
	multiple_hx711.setActiveChannels(false,true,true);
	multiple_hx711.power_up();

}

uint32_t currentMicros;
uint32_t elapsedMicros;

void loop()
{
	//uint32_t initMicros = micros();
	
	//executer.execution();
	
	//uint32_t finalMicros = micros();
	//Serial.println("cycleTime: " + (String)(finalMicros - initMicros));
	
	// wait for the chip to become ready
	if (multiple_hx711.are_xh711_ready()) {
	//if (hx711.is_ready()) {
		
		currentMicros = micros();
		multiple_hx711.update();
		elapsedMicros = micros() - currentMicros;
		
		//long Ax128ChannelValue = multiple_hx711.getAx128ChannelValue();
		long Ax64ChannelValue_0 = multiple_hx711.getAx64ChannelValue((uint16_t)0);
		long Bx32ChannelValue_0 = multiple_hx711.getBx32ChannelValue((uint16_t)0);
		long Ax64ChannelValue_1 = multiple_hx711.getAx64ChannelValue((uint16_t)1);
		long Bx32ChannelValue_1 = multiple_hx711.getBx32ChannelValue((uint16_t)1);
		
		unsigned long timeBetweenReadings = multiple_hx711.getLastElapsedTimeBetweenReadings();
		
		Serial.print("Reading ");
		//Serial.print("Ax128: ");
		//Serial.print(Ax128ChannelValue);
		Serial.print("\tHX711[0] Ax64: \t\t");
		Serial.print(Ax64ChannelValue_0);
		Serial.print("\tHX711[0] Bx32: \t\t");
		Serial.print(Bx32ChannelValue_0);
		Serial.print("\tHX711[1] Ax64: \t\t");
		Serial.print(Ax64ChannelValue_1);
		Serial.print("\tHX711[1] Bx32: \t\t");
		Serial.print(Bx32ChannelValue_1);

		Serial.print("\treadingTime (us): \t");
		Serial.print(elapsedMicros);
		Serial.print("\tTime between readings (us): \t");
		Serial.print(timeBetweenReadings);
		Serial.println();
		
		};

	if(Serial.available())
	{
		char temp = Serial.read();
		if(temp == '+')
		{
			calibration_factor += 1;
			//hx711.set_scale(calibration_factor); //Adjust to this calibration factor
		}
		else if(temp == '-')
		{
			calibration_factor -= 1;
			//hx711.set_scale(calibration_factor); //Adjust to this calibration factor
		}
		else if(temp == 'z')
		{
// 			multiple_hx711.tare_Ax128(0);
// 			multiple_hx711.tare_Ax128(1);
			;
		}
		else if(temp == 'x')
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
		else if(temp == 'd')
		{
			;
		}
	}
	
}
