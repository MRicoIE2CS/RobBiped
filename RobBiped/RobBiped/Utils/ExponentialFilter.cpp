// 
// 
// 

#include "ExponentialFilter.h"


double ExpFilter::setExpConstant(double k){
	expK = k;
}

double ExpFilter::filter(double rawValue){
	double filteredValue;
	filteredValue = lastFilteredValue*expK + (1.0-expK)*rawValue;
	lastFilteredValue = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(uint8_t rawValue){
	double filteredValue;
	filteredValue = lastFilteredValue*expK + (1.0-expK)*(double)rawValue;
	lastFilteredValue = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(int8_t rawValue){
	double filteredValue;
	filteredValue = lastFilteredValue*expK + (1.0-expK)*(double)rawValue;
	lastFilteredValue = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(uint16_t rawValue){
	double filteredValue;
	filteredValue = lastFilteredValue*expK + (1.0-expK)*(double)rawValue;
	lastFilteredValue = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(int16_t rawValue){
	double filteredValue;
	filteredValue = lastFilteredValue*expK + (1.0-expK)*(double)rawValue;
	lastFilteredValue = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(uint32_t rawValue){
	double filteredValue;
	filteredValue = lastFilteredValue*expK + (1.0-expK)*(double)rawValue;
	lastFilteredValue = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(int32_t rawValue){
	double filteredValue;
	filteredValue = lastFilteredValue*expK + (1.0-expK)*(double)rawValue;
	lastFilteredValue = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(uint64_t rawValue){
	double filteredValue;
	filteredValue = lastFilteredValue*expK + (1.0-expK)*(double)rawValue;
	lastFilteredValue = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(int64_t rawValue){
	double filteredValue;
	filteredValue = lastFilteredValue*expK + (1.0-expK)*(double)rawValue;
	lastFilteredValue = filteredValue;
	return filteredValue;
}