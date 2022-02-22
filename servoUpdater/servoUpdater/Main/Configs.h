/*
 * Configs.h
 *
 * Created: 13/02/2022 20:24:38
 *  Author: MRICO
 */ 

#ifndef _CONFIGS_h
#define _CONFIGS_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// TODO: Config objects in this file+
	
struct Configs 
{
	struct GPIO_defs {
		unsigned short squareButton = 15;
		unsigned short thinButton1 = 2;
		unsigned short thinButton2 = 4;
		unsigned short potentiometer1 = 36;
		unsigned short potentiometer2 = 39;
		}gpio;
};

#endif