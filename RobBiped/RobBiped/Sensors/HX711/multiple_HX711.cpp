/*
 * multiple_HX711.cpp
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

#include "multiple_HX711.h"

Multiple_HX711::Multiple_HX711()
{
	hx711_number_ = Configuration::hx711_number;
}

void Multiple_HX711::configure(byte _DINs[], byte pd_sck)
{
	PD_SCK_ 	= pd_sck;

	pinMode(PD_SCK_, OUTPUT);
	
	channel_selection_additional_bits_ = 3;
	
	for (uint8_t _idx = 0; _idx < hx711_number_; _idx++)
	{
		Single_HX711 hx711 = {0};
		hx711.DIN = _DINs[_idx];
		pinMode(_DINs[_idx], INPUT);
		hx711.stored_readings = {0};
		hx711.offset_per_channel = {0};
		
		v_HX711_.push_back(hx711);
	}
}

bool Multiple_HX711::are_xh711_ready()
{
	bool are_ready = true;
	for (uint8_t _idx = 0; _idx < hx711_number_; _idx++)
	{
		if (digitalRead(v_HX711_[_idx].DIN) == LOW) continue;
		are_ready = false;
		break;
	}
	return are_ready;
}

bool Multiple_HX711::update()
{
	if (are_xh711_ready())
	{
		delayMicroseconds(1);
		read_and_commute_next_channel();
		return true;
	}
	else return false;
}

void Multiple_HX711::set_sctive_channels(bool _Ax128, bool _Ax64, bool _Bx32)
{
	active_channels_.Ax128 = _Ax128;
	active_channels_.Ax64 = _Ax64;
	active_channels_.Bx32 = _Bx32;
}

Multiple_HX711::Channel Multiple_HX711::commute_next_channel(bool force_next_channel, Channel _channel)
{
	Channel nextChannel;
	
	if (force_next_channel) {nextChannel = _channel;}
	else {
		
		switch (channel_) {
			case Channel::Ax128:		// channel A, gain factor 128
			if (active_channels_.Bx32) nextChannel = Channel::Bx32;
			else if (active_channels_.Ax64) nextChannel = Channel::Ax64;
			else nextChannel = channel_;
			break;
			case Channel::Ax64:		// channel A, gain factor 64
			if (active_channels_.Bx32) nextChannel = Channel::Bx32;
			else if (active_channels_.Ax128) nextChannel = Channel::Ax128;
			else nextChannel = channel_;
			break;
			case Channel::Bx32:		// channel B, gain factor 32
			if (active_channels_.Ax128) nextChannel = Channel::Ax128;
			else if (active_channels_.Ax64) nextChannel = Channel::Ax64;
			else nextChannel = channel_;
			break;
		}
	}
	
	return nextChannel;
}

byte Multiple_HX711::set_channel_selection_bits(Channel _next_channel, bool force_next_selection, short _sel)
{
	byte selBits;
	
	if (force_next_selection) {selBits = _sel;}
	else {
		
		switch (_next_channel) {
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

void Multiple_HX711::get_DIN_pins_array(byte *_array)
{
	uint16_t _idx = 0;
	for (uint8_t _idx = 0; _idx < hx711_number_; _idx++)
	{
		*(_array + _idx) = v_HX711_[_idx].DIN;
	}
}

void Multiple_HX711::read_shiftIn(uint8_t clock_pin, byte *DIN_array, bool _readings[Configuration::hx711_number][8])
{
	uint8_t value = 0;
	uint8_t i;
	
	for (i = 0; i < 8; ++i)
	{
		digitalWrite(clock_pin, HIGH);
		delayMicroseconds(1);	// Needed to give time for the chip to change state of its output
		for (uint8_t _idx = 0; _idx < hx711_number_; _idx++)
		{
			_readings[_idx][i] = digitalRead(DIN_array[_idx]);
		}
		digitalWrite(clock_pin, LOW);
	}
}

void Multiple_HX711::construct_read(uint8_t idx_byte_sel, uint8_t bit_order, byte _arr_data[Configuration::hx711_number][3], bool _readings[Configuration::hx711_number][8])
{
	uint8_t value = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		
		if (bit_order == LSBFIRST)
		{
			uint8_t _idx = 0;
			for (uint8_t _idx = 0; _idx < hx711_number_; _idx++)
			{
				bool _read = _readings[_idx][i];
				_arr_data[_idx][idx_byte_sel] |= _read << i;
			}
		}
		else
		{
			uint8_t _idx = 0;
			for (uint8_t _idx = 0; _idx < hx711_number_; _idx++)
			{
				bool _read = _readings[_idx][i];
				_arr_data[_idx][idx_byte_sel] |= _read << (7 - i);
			}
		}
	}
}

void Multiple_HX711::read_and_commute_next_channel() 
{
	// Channel management
	Channel currentReadingChannel = channel_;
	channel_ = commute_next_channel();
	channel_selection_additional_bits_ = set_channel_selection_bits(channel_);
	
	// Auxiliary data storage objects construction
	byte arr_data[hx711_number_][3] = { 0 };

	// Pulse the clock pin 24 times to read the data.
	// [[[The reading operation needs to be fast, due to IC specifications
	// for that reason, an auxiliary array is constructed to rapidly
	// store the readings during clock signal commutation
	// That is done afterwards.]]]
	
	bool readings_array_2[hx711_number_][8];
	bool readings_array_1[hx711_number_][8];
	bool readings_array_0[hx711_number_][8];
	byte DIN_array[hx711_number_];
	
	get_DIN_pins_array(DIN_array);
	
	// Begin clock channel commutation and reading
	read_shiftIn(PD_SCK_, DIN_array, readings_array_2);
	read_shiftIn(PD_SCK_, DIN_array, readings_array_1);
	read_shiftIn(PD_SCK_, DIN_array, readings_array_0);

	// Set the channel and the gain factor for the next reading using the clock pin
	for (uint8_t i = 0; i < channel_selection_additional_bits_; i++) {
		digitalWrite(PD_SCK_, HIGH);
		digitalWrite(PD_SCK_, LOW);
	}
	// End clock channel commutation and reading
	
	// Begin data construction over obtained readings
	construct_read(2, MSBFIRST, arr_data, readings_array_2);
	construct_read(1, MSBFIRST, arr_data, readings_array_1);
	construct_read(0, MSBFIRST, arr_data, readings_array_0);

	for (uint8_t _idx = 0; _idx < hx711_number_; _idx++)
	{
		byte _data[3];
		byte filler = 0x00;
		_data[0] = arr_data[_idx][0];
		_data[1] = arr_data[_idx][1];
		_data[2] = arr_data[_idx][2];
		uint32_t value = 0;
		
		// Datasheet indicates the value is returned as a two's complement value
		// Flip all the bits
		_data[2] = ~_data[2];
		_data[1] = ~_data[1];
		_data[0] = ~_data[0];
		
		// Replicate the most significant bit to pad out a 32-bit signed integer
		if ( _data[2] & 0x80 ) {
			filler = 0xFF;
			} else if ((0x7F == _data[2]) && (0xFF == _data[1]) && (0xFF == _data[0])) {
			filler = 0xFF;
			} else {
			filler = 0x00;
		}
		
		// Construct a 32-bit signed integer
		value = ( static_cast<uint32_t>(filler) << 24
		| static_cast<uint32_t>(_data[2]) << 16
		| static_cast<uint32_t>(_data[1]) << 8
		| static_cast<uint32_t>(_data[0]) );
		
		// ... and add 1
		value = ++value;
		
		Single_HX711 *_hx711 = &v_HX711_[_idx];
		switch (currentReadingChannel)
		{
			case (Channel::Ax128):
			_hx711->stored_readings.reading_Ax128 = static_cast<int32_t>(value);
			history_append(_idx, Channel::Ax128, static_cast<int32_t>(value));
			break;
			case (Channel::Ax64):
			_hx711->stored_readings.reading_Ax64 = static_cast<int32_t>(value);
			history_append(_idx, Channel::Ax64, static_cast<int32_t>(value));
			break;
			case (Channel::Bx32):
			_hx711->stored_readings.reading_Bx32 = static_cast<int32_t>(value);
			history_append(_idx, Channel::Bx32, static_cast<int32_t>(value));
			break;
		}
	}

	uint32_t currentMicros = micros();
	last_elapsed_micros_ = currentMicros - last_reading_micros_;
	last_reading_micros_ = currentMicros;
}

void Multiple_HX711::history_append(uint16_t hx711_idx, Channel channel, int32_t _reading)
{
	if (hx711_idx >= hx711_number) return;
	Single_HX711 *_hx711 = &v_HX711_[hx711_idx];
	switch (channel)
	{
		case (Channel::Ax128):
			_hx711->history_stored_readings.v_Ax128.push_back(_reading);
			if (_hx711->history_stored_readings.v_Ax128.size() > history_length_)
			{
				_hx711->history_stored_readings.v_Ax128.erase(_hx711->history_stored_readings.v_Ax128.begin());
			}
		break;
		case (Channel::Ax64):
			_hx711->history_stored_readings.v_Ax64.push_back(_reading);
			if (_hx711->history_stored_readings.v_Ax64.size() > history_length_)
			{
				_hx711->history_stored_readings.v_Ax64.erase(_hx711->history_stored_readings.v_Ax64.begin());
			}
		break;
		case (Channel::Bx32):
			_hx711->history_stored_readings.v_Bx32.push_back(_reading);
			if (_hx711->history_stored_readings.v_Bx32.size() > history_length_)
			{
				_hx711->history_stored_readings.v_Bx32.erase(_hx711->history_stored_readings.v_Bx32.begin());
			}
		break;
	}
}

int32_t Multiple_HX711::get_Ax128_channel_value(uint16_t hx711_idx)
{
	if (hx711_idx >= hx711_number) return 0;
	Single_HX711 *_hx711 = &v_HX711_[hx711_idx];
	return _hx711->stored_readings.reading_Ax128 - _hx711->offset_per_channel.Ax128;
}

int32_t Multiple_HX711::get_Ax64_channel_value(uint16_t hx711_idx)
{
	if (hx711_idx >= hx711_number) return 0;
	Single_HX711 *_hx711 = &v_HX711_[hx711_idx];
	return _hx711->stored_readings.reading_Ax64 - _hx711->offset_per_channel.Ax64;
}

int32_t Multiple_HX711::get_Bx32_channel_value(uint16_t hx711_idx)
{
	if (hx711_idx >= hx711_number) return 0;
	Single_HX711 *_hx711 = &v_HX711_[hx711_idx];
	return _hx711->stored_readings.reading_Bx32 - _hx711->offset_per_channel.Bx32;
}

void Multiple_HX711::tare_Ax128(uint16_t hx711_idx)
{
	if (hx711_idx >= hx711_number) return;
	Single_HX711 *_hx711 = &v_HX711_[hx711_idx];
	int32_t _sum = 0;
	int32_t sizeofvector = _hx711->history_stored_readings.v_Ax128.size();
	for (uint8_t _idx = 0; _idx < sizeofvector; _idx++)
	{
		_sum += _hx711->history_stored_readings.v_Ax128[_idx];
	}
	
	if (sizeofvector == 0)
	{
		Serial.println("tare_Ax128 DIVISION BY 0 !!!");
		return;
	}
	int32_t _average = _sum / sizeofvector;
	set_offset_Ax128(hx711_idx, _average);
}

void Multiple_HX711::set_offset_Ax128(uint16_t hx711_idx, double offset)
{
	if (hx711_idx >= hx711_number) return;
	Single_HX711 *_hx711 = &v_HX711_[hx711_idx];
	_hx711->offset_per_channel.Ax128 = offset;
}

void Multiple_HX711::tare_Ax64(uint16_t hx711_idx)
{
	if (hx711_idx >= hx711_number) return;
	Single_HX711 *_hx711 = &v_HX711_[hx711_idx];
	int32_t _sum = 0;
	short sizeofvector = _hx711->history_stored_readings.v_Ax64.size();
	for (uint8_t _idx = 0; _idx < sizeofvector; _idx++)
	{
		_sum += _hx711->history_stored_readings.v_Ax64[_idx];
	}
	if (sizeofvector == 0)
	{
		Serial.println("tare_Ax64 DIVISION BY 0 !!!");
	}
	int32_t _average = _sum / sizeofvector;
	set_offset_Ax64(hx711_idx, _average);
}

void Multiple_HX711::set_offset_Ax64(uint16_t hx711_idx, double offset)
{
	if (hx711_idx >= hx711_number) return;
	Single_HX711 *_hx711 = &v_HX711_[hx711_idx];
	_hx711->offset_per_channel.Ax64 = offset;
}

void Multiple_HX711::tare_Bx32(uint16_t hx711_idx)
{
	if (hx711_idx >= hx711_number) return;
	Single_HX711 *_hx711 = &v_HX711_[hx711_idx];
	int32_t _sum = 0;
	int32_t sizeofvector = _hx711->history_stored_readings.v_Bx32.size();
	for (uint8_t _idx = 0; _idx < sizeofvector; _idx++)
	{
		_sum += _hx711->history_stored_readings.v_Bx32[_idx];
	}
	if (sizeofvector == 0)
	{
		Serial.println("tare_Bx32 DIVISION BY 0 !!!");
		return;
	}
	int32_t _average = _sum / sizeofvector;
	set_offset_Bx32(hx711_idx, _average);
}

void Multiple_HX711::set_offset_Bx32(uint16_t hx711_idx, double offset)
{
	if (hx711_idx >= hx711_number) return;
	Single_HX711 *_hx711 = &v_HX711_[hx711_idx];
	_hx711->offset_per_channel.Bx32 = offset;
}

uint32_t Multiple_HX711::get_last_elapsed_time_between_readings()
{
	return last_elapsed_micros_;
}

void Multiple_HX711::power_down()
{
	digitalWrite(PD_SCK_, LOW);
	digitalWrite(PD_SCK_, HIGH);
}

void Multiple_HX711::power_up()
{
	digitalWrite(PD_SCK_, LOW);
}
