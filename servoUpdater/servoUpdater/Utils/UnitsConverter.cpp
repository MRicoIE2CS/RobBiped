// 
// 
// 

#include "UnitsConverter.h"


double UnitsConvert::Angle::degToRad(double _ang){
	double ang_rad = (_ang* PI) /180.0 ;
	return ang_rad;
}

double UnitsConvert::Angle::radToDeg(double _ang){
	double ang_deg = _ang /PI * 180.0;
	return ang_deg;
}

