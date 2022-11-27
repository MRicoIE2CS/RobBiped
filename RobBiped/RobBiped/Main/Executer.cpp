/*
 * Executer.cpp
 *
 * Created: 27/01/2022 11:28:19
 *  Author: MRICO
 */ 

#include "Executer.h"

void Executer::init()
{
	setup();
	
	//______TASKS CONFIGURATION_____//
	
	userInput.setExecutionPeriod(I_PeriodicTask::execType::inMillis, 5);
	
	signalGenerator_0.setExecutionPeriod(I_PeriodicTask::execType::inMillis, 20);
	signalGenerator_0.configureSignal(SignalGenerator::SignalType::sine,500,1,0,0);
	signalGenerator_0.init();
	
	servoUpdater.setExecutionPeriod(I_PeriodicTask::execType::inMillis,20);	
	servoUpdater.init();
	
	forceSensorsManager.setExecutionPeriod(I_PeriodicTask::execType::inMillis, 5);
	forceSensorsManager.init();
	
	gyroscopeAccelerometerManager.setExecutionPeriod(I_PeriodicTask::execType::inMillis, 4);
	gyroscopeAccelerometerManager.init();
	
	// END TASKS CONFIGURATION
}

void Executer::inputs()
{
	if (userInput.getExecutionFlag()) userInput.update();
	
	if (gyroscopeAccelerometerManager.getExecutionFlag())
	{
		bool updated = gyroscopeAccelerometerManager.update();
	}
	
	if (forceSensorsManager.getExecutionFlag())
	{
		bool updated = forceSensorsManager.update();
		
		if (updated) {
			// DEBUG:
			
			// 			Serial.println("Reading____________________________");
			// 			Serial.print("LeftFoot_LeftFrontSensor: \t\t");
			// 			Serial.print(forceSensorsManager.getValue_LeftFoot_LeftFrontSensor());
			// 			Serial.print("\tLeftFoot_RightFrontSensor: \t\t");
			// 			Serial.println(forceSensorsManager.getValue_LeftFoot_RightFrontSensor());
			// 			Serial.print("LeftFoot_LeftBackSensor: \t\t");
			// 			Serial.print(forceSensorsManager.getValue_LeftFoot_LeftBackSensor());
			// 			Serial.print("\tLeftFoot_RightBackSensor: \t\t");
			// 			Serial.println(forceSensorsManager.getValue_LeftFoot_RightBackSensor());
			//
			//
			// 			Serial.print("RightFoot_LeftFrontSensor: \t\t");
			// 			Serial.print(forceSensorsManager.getValue_RightFoot_LeftFrontSensor());
			// 			Serial.print("\tRightFoot_RightFrontSensor: \t\t");
			// 			Serial.println(forceSensorsManager.getValue_RightFoot_RightFrontSensor());
			// 			Serial.print("RightFoot_LeftBackSensor: \t\t");
			// 			Serial.print(forceSensorsManager.getValue_RightFoot_LeftBackSensor());
			// 			Serial.print("\tRightFoot_RightBackSensor: \t\t");
			// 			Serial.println(forceSensorsManager.getValue_RightFoot_RightBackSensor());
			//
			// 			Serial.println("\tTime between readings (us): \t");
			// 			Serial.print(forceSensorsManager.getLastElapsedTimeBetweenReadings());
			// 			Serial.println();
			//
			// 			if(Serial.available())
			// 			{
			// 				char temp = Serial.read();
			// 				if(temp == 'x')
			// 				{
			// 					Serial.println("TARE");
			// 					forceSensorsManager.tare_LeftFoot();
			// 				}
			// 				else if(temp == 'c')
			// 				{
			// 					Serial.println("TARE");
			// 					forceSensorsManager.tare_RightFoot();
			// 				}
			// 			}
			
			// END DEBUG
		}
	}
}

void Executer::mainExecution()
{
	
	
	
}

void Executer::outputs()
{
	if (servoUpdater.getExecutionFlag())
	{
		// INIT Example of using potentiometer to command servos
		uint16_t pot1Val = userInput.getAnalogValue(UserInput::AnalogInputList::potentiometer2);
		
		double readingAngle_0;
		if (pot1Val < 1000){
			readingAngle_0 = 0;
		}
		else if (pot1Val > 3000){
			readingAngle_0 = PI;
		}
		else {
			readingAngle_0 = (double)(pot1Val - 1000) / (double)2000 * PI;
		}
		double nextAngle_0 = readingAngle_0 - HALF_PI;
		// END Example
		
		// Servo setpoint assignation
		servoUpdater.setAngleToServo(0,nextAngle_0);
		servoUpdater.setAngleToServo(1,nextAngle_0);
		servoUpdater.setAngleToServo(2,nextAngle_0);
		servoUpdater.setAngleToServo(3,nextAngle_0);
		servoUpdater.setAngleToServo(4,nextAngle_0);
		servoUpdater.setAngleToServo(5,nextAngle_0);
		servoUpdater.setAngleToServo(6,nextAngle_0);
		servoUpdater.setAngleToServo(7,nextAngle_0);
		servoUpdater.setAngleToServo(8,nextAngle_0);
		servoUpdater.setAngleToServo(9,nextAngle_0);
		servoUpdater.setAngleToServo(10,nextAngle_0);
		servoUpdater.setAngleToServo(11,nextAngle_0);
		servoUpdater.setAngleToServo(12,nextAngle_0);
		servoUpdater.setAngleToServo(13,nextAngle_0);
		servoUpdater.setAngleToServo(14,nextAngle_0);
		servoUpdater.setAngleToServo(15,nextAngle_0);
		
		// Servo setpoint command
		servoUpdater.update(userInput);
	}
}

void Executer::execution()
{
	inputs();
	
	mainExecution();
	
	outputs();
}