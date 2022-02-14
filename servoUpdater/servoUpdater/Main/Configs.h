/*
 * Configs.h
 *
 * Created: 13/02/2022 20:24:38
 *  Author: MRICO
 */ 


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