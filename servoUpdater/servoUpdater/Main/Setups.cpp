/*
 * Setups.cpp
 *
 * Created: 13/02/2022 20:04:17
 *  Author: MRICO
 */ 

#include "Executer.h"

void Executer::IOs_setup(){
	
	pinMode(config.gpio.squareButton,INPUT);	// Square button
	pinMode(config.gpio.thinButton1,INPUT);	// Thin button 1
	pinMode(config.gpio.thinButton2,INPUT);	// Thin button 2
	
	adcAttachPin(config.gpio.potentiometer1);	// Potentiometer 1
	adcAttachPin(config.gpio.potentiometer2);	// Potentiometer 2
	
}