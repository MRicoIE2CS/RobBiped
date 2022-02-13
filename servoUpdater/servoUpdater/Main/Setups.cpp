/*
 * Setups.cpp
 *
 * Created: 13/02/2022 20:04:17
 *  Author: MRICO
 */ 

#include "Executer.h"

void Executer::IOs_setup(){
	
	pinMode(15,INPUT);	// Square button
	pinMode(2,INPUT);	// Thin button 1
	pinMode(4,INPUT);	// Thin button 2
	
	adcAttachPin(36);	// Potentiometer 1
	adcAttachPin(39);	// Potentiometer 2
	
}