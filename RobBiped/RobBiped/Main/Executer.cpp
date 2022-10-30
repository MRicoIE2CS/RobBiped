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
	
	// END TASKS CONFIGURATION
}

void Executer::execution()
{
	// INPUTS:
	
	if (userInput.getExecutionFlag()) userInput.update();
	
	if (forceSensorsManager.getExecutionFlag()) 
	{
		bool updated = forceSensorsManager.update();
	
		if (updated) {
			// DEBUG:
		
			Serial.println("Reading____________________________");
			Serial.print("LeftFoot_LeftFrontSensor: \t\t");
			Serial.print(forceSensorsManager.getValue_LeftFoot_LeftFrontSensor());
			if (abs(forceSensorsManager.getValue_LeftFoot_LeftFrontSensor()) > 4000) {
				Serial.println("\n MALA! _____________________________________LeftFoot_LeftFrontSensor_________________________________________________");
				//delay(2000);
			}
			Serial.print("\tLeftFoot_RightFrontSensor: \t\t");
			Serial.println(forceSensorsManager.getValue_LeftFoot_RightFrontSensor());
			if (abs(forceSensorsManager.getValue_LeftFoot_RightFrontSensor()) > 4000) {
				Serial.println("\n MALA! _____________________________________LeftFoot_RightFrontSensor_________________________________________________");
				//delay(2000);
			}
			Serial.print("LeftFoot_LeftBackSensor: \t\t");
			Serial.print(forceSensorsManager.getValue_LeftFoot_LeftBackSensor());
 			if (abs(forceSensorsManager.getValue_LeftFoot_LeftBackSensor()) > 4000) Serial.println("\n MALA! _____________________________________getValue_LeftFoot_LeftBackSensor_________________________________________________");
			Serial.print("\tLeftFoot_RightBackSensor: \t\t");
			Serial.println(forceSensorsManager.getValue_LeftFoot_RightBackSensor());
 			if (abs(forceSensorsManager.getValue_LeftFoot_RightBackSensor()) > 4000) Serial.println("\n MALA! _____________________________________getValue_LeftFoot_RightBackSensor_________________________________________________");
			
			
			
// 			Serial.println((int32_t)forceSensorsManager.getValue_LeftFoot_LeftFrontSensor(),BIN);
// 			Serial.println((int32_t)forceSensorsManager.getValue_LeftFoot_RightFrontSensor(),BIN);
// 			Serial.println((int32_t)forceSensorsManager.getValue_LeftFoot_LeftBackSensor(),BIN);
// 			Serial.println((int32_t)forceSensorsManager.getValue_LeftFoot_RightBackSensor(),BIN);
		
			Serial.print("RightFoot_LeftFrontSensor: \t\t");
			Serial.print(forceSensorsManager.getValue_RightFoot_LeftFrontSensor());
 			if (abs(forceSensorsManager.getValue_RightFoot_LeftFrontSensor()) > 4000) Serial.println("\n MALA! _____________________________________getValue_RightFoot_LeftFrontSensor_________________________________________________");
			Serial.print("\tRightFoot_RightFrontSensor: \t\t");
			Serial.println(forceSensorsManager.getValue_RightFoot_RightFrontSensor());
 			if (abs(forceSensorsManager.getValue_RightFoot_RightFrontSensor()) > 4000) Serial.println("\n MALA! _____________________________________getValue_RightFoot_RightFrontSensor_________________________________________________");
			Serial.print("RightFoot_LeftBackSensor: \t\t");
			Serial.print(forceSensorsManager.getValue_RightFoot_LeftBackSensor());
 			if (abs(forceSensorsManager.getValue_RightFoot_LeftBackSensor()) > 4000) Serial.println("\n MALA! _____________________________________getValue_RightFoot_LeftBackSensor_________________________________________________");
			Serial.print("\tRightFoot_RightBackSensor: \t\t");
			Serial.println(forceSensorsManager.getValue_RightFoot_RightBackSensor());
 			if (abs(forceSensorsManager.getValue_RightFoot_RightBackSensor()) > 4000) Serial.println("\n MALA! _____________________________________getValue_RightFoot_RightBackSensor_________________________________________________");
			
// 			Serial.println((int32_t)forceSensorsManager.getValue_RightFoot_LeftFrontSensor(),BIN);
// 			Serial.println((int32_t)forceSensorsManager.getValue_RightFoot_RightFrontSensor(),BIN);
// 			Serial.println((int32_t)forceSensorsManager.getValue_RightFoot_LeftBackSensor(),BIN);
// 			Serial.println((int32_t)forceSensorsManager.getValue_RightFoot_RightBackSensor(),BIN);

			Serial.println("\tTime between readings (us): \t");
			Serial.print(forceSensorsManager.getLastElapsedTimeBetweenReadings());
			Serial.println();
		
			if(Serial.available())
			{
				char temp = Serial.read();
				if(temp == 'x')
				{
					Serial.println("TARE");
					forceSensorsManager.tare_LeftFoot();
				}
				else if(temp == 'c')
				{
					Serial.println("TARE");
					forceSensorsManager.tare_RightFoot();
				}
			}
		
			// END DEBUG
		}
	}
	
	// MAIN EXECUTION:
	
	//if (userInput.getDigitalValue(UserInput::DigitalInputList::squareButton)) servoUpdater.changeState();	// run/sleep to servos
	
	if (signalGenerator_0.getExecutionFlag()) {
		
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
		nextAngle_0 = 0.0;
		//nextAngle_0 = HALF_PI + 3*HALF_PI/4 * signalGenerator_0.generateTrajectory();
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
	}
	
	// OUTPUTS:
	
	if (servoUpdater.getExecutionFlag()) servoUpdater.update(userInput);
}