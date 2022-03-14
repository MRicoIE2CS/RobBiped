

/*
 * RobBiped.ino
 *
 * Created: 1/28/2022 8:02:12 PM
 * Author: MRICO
 */ 

#include "Main/Executer.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "HX711.h"

#define SCKpin 18
#define DOpin 19
HX711 scale(DOpin, SCKpin);
float calibration_factor = 2230; // this calibration factor is adjusted according to my load cell
float units;
float ounces;

Executer executer;

void setup()
{
	Serial.begin(500000);
	
	//executer.init();
	
	Serial.println("Initial delay...");
	delay(3000);
	
	Serial.println("HX711 calibration sketch");
	Serial.println("Remove all weight from scale");
	Serial.println("After readings begin, place known weight on scale");
	Serial.println("Press + or a to increase calibration factor");
	Serial.println("Press - or z to decrease calibration factor");

	scale.set_scale();
	scale.tare();  //Reset the scale to 0

	long zero_factor = scale.read_average(); //Get a baseline reading
	Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
	Serial.println(zero_factor);
	
	

}

void loop()
{
	//uint32_t initMicros = micros();
	
	//executer.execution();
	
	//uint32_t finalMicros = micros();
	//Serial.println("cycleTime: " + (String)(finalMicros - initMicros));
	
	scale.set_scale(calibration_factor); //Adjust to this calibration factor

	Serial.print("Reading: ");
	units = scale.get_units(), 10;
	if (units < 0)
	{
		units = 0.00;
	}
	ounces = units * 0.035274;
	Serial.print(units);
	Serial.print(" grams");
	Serial.print(" calibration_factor: ");
	Serial.print(calibration_factor);
	Serial.println();

	if(Serial.available())
	{
		char temp = Serial.read();
		if(temp == '+' || temp == 'a')
		calibration_factor += 1;
		else if(temp == '-' || temp == 'z')
		calibration_factor -= 1;
	}
	
}
