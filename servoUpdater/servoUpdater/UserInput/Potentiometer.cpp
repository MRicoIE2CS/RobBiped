// 
// 
// 

#include "Potentiometer.h"


void Potentiometer::setup(uint8_t _pin, double _Kfilter){
	
	HWpin = _pin;
	adcAttachPin(HWpin);
	expFilter.setExpConstant(_Kfilter);
}

uint16_t Potentiometer::readHWvalue(){
	
	value = expFilter.filter(analogRead(HWpin));
}

uint16_t Potentiometer::getValue(){
	
	return value;
}