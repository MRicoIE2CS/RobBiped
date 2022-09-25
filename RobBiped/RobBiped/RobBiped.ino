

/*
 * RobBiped.ino
 *
 * Created: 1/28/2022 8:02:12 PM
 * Author: MRICO
 */ 

#include "Main/Executer.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//#include "HX711.h"
#include "Sensors/HX711/HX711.h"

#define SCKpin 18
#define DOpin 19
HX711 hx711(DOpin, SCKpin);
float calibration_factor = 1;
float units;
float ounces;

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
	hx711.setActiveChannels(false,true,true);
	hx711.power_up();

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
	if (hx711.is_ready()) {
		
		currentMicros = micros();
		hx711.update();
		elapsedMicros = micros() - currentMicros;
		
		long Ax128ChannelValue = hx711.getAx128ChannelValue();
		long Ax64ChannelValue = hx711.getAx64ChannelValue();
		long Bx32ChannelValue = hx711.getBx32ChannelValue();
		
		unsigned long timeBetweenReadings = hx711.getLastElapsedTimeBetweenReadings();
		
		Serial.print("Reading Ax128: ");
		Serial.print(Ax128ChannelValue);
		Serial.print("; Ax64: ");
		Serial.print(Ax64ChannelValue);
		Serial.print("; Bx32: ");
		Serial.print(Bx32ChannelValue);
		Serial.print("; calibration_factor: ");
		Serial.print(calibration_factor);
		Serial.print("; readingTime (us): ");
		Serial.print(elapsedMicros);
		Serial.print("; Time between readings (us): ");
		Serial.print(timeBetweenReadings);
		Serial.println();
		
		};
	
	

	if(Serial.available())
	{
		char temp = Serial.read();
		if(temp == '+')
		{
			calibration_factor += 1;
			hx711.set_scale(calibration_factor); //Adjust to this calibration factor
		}
		else if(temp == '-')
		{
			calibration_factor -= 1;
			hx711.set_scale(calibration_factor); //Adjust to this calibration factor
		}
		else if(temp == 'z')
		hx711.tare_Ax128();
		else if(temp == 'x')
		hx711.tare_Ax64();
		else if(temp == 'c')
		hx711.tare_Bx32();
	}
	
}
