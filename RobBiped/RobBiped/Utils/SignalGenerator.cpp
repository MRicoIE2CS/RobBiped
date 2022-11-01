// 
// 
// 

#include "SignalGenerator.h"


double SignalGenerator::generateTrajectory(){
	
	unsigned long currentMillis = millis();
	
	double nextOutput = offset + amplitude * sin( (double)currentMillis/(double)period_ms + (double)phaseShift) ;
	
	lastTimeExecuted = currentMillis;
	
	return nextOutput;
}

void SignalGenerator::configureSignal(SignalGenerator::SignalType _type, unsigned int _period, unsigned int _amplitude, unsigned int _offset, unsigned int _phaseShift){
	signalType = _type;
	period_ms = _period;
	amplitude = _amplitude;
	offset = _offset;
	phaseShift = _phaseShift;
}

void SignalGenerator::init(){
	
	lastCalculatedTime = millis();
	last_output = 0;
}

void SignalGenerator::update(){
	
}