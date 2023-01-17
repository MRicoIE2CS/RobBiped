/*
 * HX711.cpp
 *
 * Copyright 2023 Mikel Rico Abajo (MRicoIE2C)

 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "HX711.h"

HX711::HX711(byte dout, byte pd_sck, byte gain) {
	PD_SCK 	= pd_sck;
	DOUT 	= dout;

	pinMode(PD_SCK, OUTPUT);
	pinMode(DOUT, INPUT);

	//set_gain(gain);
}

HX711::~HX711() {

}

bool HX711::is_ready() {
	return digitalRead(DOUT) == LOW;
}

void HX711::update(){
	if (is_ready()){
		CombinedOutputData newReading = readAndConmuteNextChannel();
		switch (newReading._channel)
		{
			case (Channel::Ax128):
			storedReadings.ReadingAx128 = newReading._valueRead;
			break;
			case (Channel::Ax64):
			storedReadings.ReadingAx64 = newReading._valueRead;
			break;
			case (Channel::Bx32):
			storedReadings.ReadingBx32 = newReading._valueRead;
			break;
		}
	}
}

void HX711::setActiveChannels(bool _Ax128, bool _Ax64, bool _Bx32){
	activeChannels.Ax128 = _Ax128;
	activeChannels.Ax64 = _Ax64;
	activeChannels.Bx32 = _Bx32;
}

HX711::Channel HX711::conmuteNextChannel(bool forceNextChannel, Channel _channel) {
	Channel nextChannel;
	
	if (forceNextChannel) {nextChannel = _channel;}
	else {
		
		switch (channel) {
			case Channel::Ax128:		// channel A, gain factor 128
			if (activeChannels.Bx32) nextChannel = Channel::Bx32;
			else if (activeChannels.Ax64) nextChannel = Channel::Ax64;
			else nextChannel = channel;
			break;
			case Channel::Ax64:		// channel A, gain factor 64
			if (activeChannels.Bx32) nextChannel = Channel::Bx32;
			else if (activeChannels.Ax128) nextChannel = Channel::Ax128;
			else nextChannel = channel;
			break;
			case Channel::Bx32:		// channel B, gain factor 32
			if (activeChannels.Ax128) nextChannel = Channel::Ax128;
			else if (activeChannels.Ax64) nextChannel = Channel::Ax64;
			else nextChannel = channel;
			break;
		}
	}
	
	return nextChannel;
}

byte HX711::setChannelSelectionBits(Channel _nextChannel, bool forceNextSelection, short _sel) {
	byte selBits;
	
	if (forceNextSelection) {selBits = _sel;}
	else {
		
		switch (_nextChannel) {
			case Channel::Ax128:
			selBits = 1;
			break;
			case Channel::Ax64:
			selBits = 3;
			break;
			case Channel::Bx32:
			selBits = 2;
			break;
		}
	}
	
	return selBits;
}

HX711::CombinedOutputData HX711::readAndConmuteNextChannel() {
	
	CombinedOutputData outData;
	
	Channel nextChannel;
	nextChannel = conmuteNextChannel();
	outData._channel = channel;
	channel = nextChannel;
	ChSelBits = setChannelSelectionBits(nextChannel);
	

	unsigned long value = 0;
	byte data[3] = { 0 };
	byte filler = 0x00;

	// pulse the clock pin 24 times to read the data
	data[2] = shiftIn(DOUT, PD_SCK, MSBFIRST);
	data[1] = shiftIn(DOUT, PD_SCK, MSBFIRST);
	data[0] = shiftIn(DOUT, PD_SCK, MSBFIRST);

	// set the channel and the gain factor for the next reading using the clock pin
	for (unsigned int i = 0; i < ChSelBits; i++) {
		digitalWrite(PD_SCK, HIGH);
		digitalWrite(PD_SCK, LOW);
	}

	// Datasheet indicates the value is returned as a two's complement value
	// Flip all the bits
	data[2] = ~data[2];
	data[1] = ~data[1];
	data[0] = ~data[0];

	// Replicate the most significant bit to pad out a 32-bit signed integer
	if ( data[2] & 0x80 ) {
		filler = 0xFF;
		} else if ((0x7F == data[2]) && (0xFF == data[1]) && (0xFF == data[0])) {
		filler = 0xFF;
		} else {
		filler = 0x00;
	}

	// Construct a 32-bit signed integer
	value = ( static_cast<unsigned long>(filler) << 24
	| static_cast<unsigned long>(data[2]) << 16
	| static_cast<unsigned long>(data[1]) << 8
	| static_cast<unsigned long>(data[0]) );

	// ... and add 1
	outData._valueRead = static_cast<long>(++value);
	
	unsigned long currentMicros = micros();
	lastElapsedMicros = currentMicros - lastReadingMicros;
	lastReadingMicros = currentMicros;
	
	return outData;
	
}

long HX711::getAx128ChannelValue(){
	return storedReadings.ReadingAx128 - offsetPerChannel.Ax128;
}

long HX711::getAx64ChannelValue(){
	return storedReadings.ReadingAx64 - offsetPerChannel.Ax64;
}

long HX711::getBx32ChannelValue(){
	return storedReadings.ReadingBx32 - offsetPerChannel.Bx32;
}

void HX711::tare_Ax128(byte times){
	set_gain(128);
	double sum = read_average(times);
	set_offset_Ax128(sum);
}

void HX711::set_offset_Ax128(double offset){
	offsetPerChannel.Ax128 = offset;
}

void HX711::tare_Ax64(byte times){
	set_gain(64);
	double sum = read_average(times);
	set_offset_Ax64(sum);
}

void HX711::set_offset_Ax64(double offset){
	offsetPerChannel.Ax64 = offset;
}

void HX711::tare_Bx32(byte times){
	set_gain(32);
	double sum = read_average(times);
	set_offset_Bx32(sum);
}

void HX711::set_offset_Bx32(double offset){
	offsetPerChannel.Bx32 = offset;
}

unsigned long HX711::getLastElapsedTimeBetweenReadings(){
	return lastElapsedMicros;
}

void HX711::set_gain(byte gain) {
	switch (gain) {
		case 128:		// channel A, gain factor 128
		GAIN = 1;
		break;
		case 64:		// channel A, gain factor 64
		GAIN = 3;
		break;
		case 32:		// channel B, gain factor 32
		GAIN = 2;
		break;
	}

	digitalWrite(PD_SCK, LOW);
	read();
}

long HX711::read() {
	// wait for the chip to become ready
	while (!is_ready());

	unsigned long value = 0;
	byte data[3] = { 0 };
	byte filler = 0x00;

	// pulse the clock pin 24 times to read the data
	data[2] = shiftIn(DOUT, PD_SCK, MSBFIRST);
	data[1] = shiftIn(DOUT, PD_SCK, MSBFIRST);
	data[0] = shiftIn(DOUT, PD_SCK, MSBFIRST);

	// set the channel and the gain factor for the next reading using the clock pin
	for (unsigned int i = 0; i < GAIN; i++) {
		digitalWrite(PD_SCK, HIGH);
		digitalWrite(PD_SCK, LOW);
	}

	// Datasheet indicates the value is returned as a two's complement value
	// Flip all the bits
	data[2] = ~data[2];
	data[1] = ~data[1];
	data[0] = ~data[0];

	// Replicate the most significant bit to pad out a 32-bit signed integer
	if ( data[2] & 0x80 ) {
		filler = 0xFF;
		} else if ((0x7F == data[2]) && (0xFF == data[1]) && (0xFF == data[0])) {
		filler = 0xFF;
		} else {
		filler = 0x00;
	}

	// Construct a 32-bit signed integer
	value = ( static_cast<unsigned long>(filler) << 24
	| static_cast<unsigned long>(data[2]) << 16
	| static_cast<unsigned long>(data[1]) << 8
	| static_cast<unsigned long>(data[0]) );

	// ... and add 1
	return static_cast<long>(++value);
}

long HX711::read_average(byte times) {
	long sum = 0;
	for (byte i = 0; i < times; i++) {
		sum += read();
	}
	return sum / times;
}

double HX711::get_value(byte times) {
	return read_average(times) - OFFSET;
}

float HX711::get_units(byte times) {
	return get_value(times) / SCALE;
}

void HX711::tare(byte times) {
	double sum = read_average(times);
	set_offset(sum);
}

void HX711::set_scale(float scale) {
	SCALE = scale;
}

float HX711::get_scale() {
	return SCALE;
}

void HX711::set_offset(long offset) {
	OFFSET = offset;
}

long HX711::get_offset() {
	return OFFSET;
}

void HX711::power_down() {
	digitalWrite(PD_SCK, LOW);
	digitalWrite(PD_SCK, HIGH);
}

void HX711::power_up() {
	digitalWrite(PD_SCK, LOW);
}
