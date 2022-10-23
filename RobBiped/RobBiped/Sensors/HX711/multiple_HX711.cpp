
/*
 * multiple_HX711.cpp
 *
 * Created: 20/03/2022
 * Author: MRICO
 */ 

#include "multiple_HX711.h"

Multiple_HX711::Multiple_HX711()
{
	_hx711_number = hx711_number;
}

Multiple_HX711::~Multiple_HX711()
{

}

void Multiple_HX711::configure(byte _DINs[], byte pd_sck)
{
	PD_SCK 	= pd_sck;

	pinMode(PD_SCK, OUTPUT);
	
	activeChannels.Ax128 = false;
	activeChannels.Ax64 = true;
	activeChannels.Bx32 = true;
	ChSelBits = 3;
	
	for (uint8_t _idx = 0; _idx < _hx711_number; _idx++)
	{
		Single_HX711 hx711 = {0};
		hx711.DIN = _DINs[_idx];
		pinMode(_DINs[_idx], INPUT);
		hx711.storedReadings = {0};
		hx711.offsetPerChannel = {0};
		
		v_HX711.push_back(hx711);
	}
}

bool Multiple_HX711::are_xh711_ready()
{
	bool are_ready = true;
	for (uint8_t _idx = 0; _idx < _hx711_number; _idx++)
	{
		if (digitalRead(v_HX711[_idx].DIN) == LOW) continue;
		are_ready = false;
		break;
	}
	return are_ready;
}

bool Multiple_HX711::update()
{
	if (are_xh711_ready())
	{
		readAndCommuteNextChannel();
		return true;
	}
	else return false;
}

void Multiple_HX711::setActiveChannels(bool _Ax128, bool _Ax64, bool _Bx32)
{
	activeChannels.Ax128 = _Ax128;
	activeChannels.Ax64 = _Ax64;
	activeChannels.Bx32 = _Bx32;
}

Multiple_HX711::Channel Multiple_HX711::commuteNextChannel(bool forceNextChannel, Channel _channel)
{
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

byte Multiple_HX711::setChannelSelectionBits(Channel _nextChannel, bool forceNextSelection, short _sel)
{
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

void Multiple_HX711::get_DIN_pins_array(byte *_array)
{
	uint16_t _idx = 0;
	for (uint8_t _idx = 0; _idx < _hx711_number; _idx++)
	{
		*(_array + _idx) = v_HX711[_idx].DIN;
	}
}

void Multiple_HX711::read_shiftIn(uint8_t clockPin, byte *DIN_array, bool _readings[Configuration::hx711_number][8])
{
	uint8_t value = 0;
	uint8_t i;
	

	for (i = 0; i < 8; ++i)
	{
		digitalWrite(clockPin, HIGH);
		delayMicroseconds(2);	// Needed to give time for the chip to change state of its output
		for (uint8_t _idx = 0; _idx < _hx711_number; _idx++)
		{
			_readings[_idx][i] = digitalRead(DIN_array[_idx]);
		}
		digitalWrite(clockPin, LOW);
	}
	
}

void Multiple_HX711::construct_read(uint8_t idx_byte_sel, uint8_t clockPin, uint8_t bitOrder, byte _arr_data[Configuration::hx711_number][3], bool _readings[Configuration::hx711_number][8])
{
	uint8_t value = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		
		if (bitOrder == LSBFIRST)
		{
			uint8_t _idx = 0;
			for (uint8_t _idx = 0; _idx < _hx711_number; _idx++)
			{
				bool _read = _readings[_idx][i];
				_arr_data[_idx][idx_byte_sel] |= _read << i;
			}
		}
		else
		{
			uint8_t _idx = 0;
			for (uint8_t _idx = 0; _idx < _hx711_number; _idx++)
			{
				bool _read = _readings[_idx][i];
				_arr_data[_idx][idx_byte_sel] |= _read << (7 - i);
			}
		}
	}
}

void Multiple_HX711::readAndCommuteNextChannel() 
{
	// Channel management
	Channel currentReadingChannel = channel;
	channel = commuteNextChannel();
	ChSelBits = setChannelSelectionBits(channel);
	
	// Auxiliary data storage objects construction
	byte arr_data[_hx711_number][3] = { 0 };

	// Pulse the clock pin 24 times to read the data.
	// [[[The reading operation needs to be fast, due to IC specifications
	// for that reason, an auxiliary array is constructed to rapidly
	// store the readings during clock signal commutation
	// That is done afterwards.]]]
	
	bool readings_array_2[_hx711_number][8];
	bool readings_array_1[_hx711_number][8];
	bool readings_array_0[_hx711_number][8];
	byte DIN_array[_hx711_number];
	
	get_DIN_pins_array(DIN_array);
	
	// Begin clock channel commutation and reading
	read_shiftIn(PD_SCK, DIN_array, readings_array_2);
	read_shiftIn(PD_SCK, DIN_array, readings_array_1);
	read_shiftIn(PD_SCK, DIN_array, readings_array_0);

	// Set the channel and the gain factor for the next reading using the clock pin
	for (unsigned int i = 0; i < ChSelBits; i++) {
		digitalWrite(PD_SCK, HIGH);
		digitalWrite(PD_SCK, LOW);
	}
	// End clock channel commutation and reading
	
	// Begin data construction over obtained readings
	construct_read(2, PD_SCK, MSBFIRST, arr_data, readings_array_2);
	construct_read(1, PD_SCK, MSBFIRST, arr_data, readings_array_1);
	construct_read(0, PD_SCK, MSBFIRST, arr_data, readings_array_0);

	for (uint8_t _idx = 0; _idx < _hx711_number; _idx++)
	{
		byte _data[3];
		byte filler = 0x00;
		_data[0] = arr_data[_idx][0];
		_data[1] = arr_data[_idx][1];
		_data[2] = arr_data[_idx][2];
		//_data = static_cast<byte[3]>(_iterator);
		unsigned long value = 0;
		
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
		value = ( static_cast<unsigned long>(filler) << 24
		| static_cast<unsigned long>(_data[2]) << 16
		| static_cast<unsigned long>(_data[1]) << 8
		| static_cast<unsigned long>(_data[0]) );
		
		// ... and add 1
		value = ++value;
		
		Single_HX711 *_hx711 = &v_HX711[_idx];
		switch (currentReadingChannel)
		{
			case (Channel::Ax128):
			_hx711->storedReadings.ReadingAx128 = static_cast<long>(value);
			historyAppend(_idx, Channel::Ax128, static_cast<long>(value));
			break;
			case (Channel::Ax64):
			_hx711->storedReadings.ReadingAx64 = static_cast<long>(value);
			historyAppend(_idx, Channel::Ax64, static_cast<long>(value));
			break;
			case (Channel::Bx32):
			_hx711->storedReadings.ReadingBx32 = static_cast<long>(value);
			historyAppend(_idx, Channel::Bx32, static_cast<long>(value));
			break;
		}
	}

	unsigned long currentMicros = micros();
	lastElapsedMicros = currentMicros - lastReadingMicros;
	lastReadingMicros = currentMicros;
}

void Multiple_HX711::historyAppend(uint16_t hx711_idx, Channel channel, long _reading)
{
	Single_HX711 *_hx711 = &v_HX711[hx711_idx];
	switch (channel)
	{
		case (Channel::Ax128):
			_hx711->historyStoredReadings.v_Ax128.push_back(_reading);
			if (_hx711->historyStoredReadings.v_Ax128.size() > historyLength)
			{
				_hx711->historyStoredReadings.v_Ax128.erase(_hx711->historyStoredReadings.v_Ax128.begin());
			}
		break;
		case (Channel::Ax64):
			_hx711->historyStoredReadings.v_Ax64.push_back(_reading);
			if (_hx711->historyStoredReadings.v_Ax64.size() > historyLength)
			{
				_hx711->historyStoredReadings.v_Ax64.erase(_hx711->historyStoredReadings.v_Ax64.begin());
			}
		break;
		case (Channel::Bx32):
			_hx711->historyStoredReadings.v_Bx32.push_back(_reading);
			if (_hx711->historyStoredReadings.v_Bx32.size() > historyLength)
			{
				_hx711->historyStoredReadings.v_Bx32.erase(_hx711->historyStoredReadings.v_Bx32.begin());
			}
		break;
	}
}

long Multiple_HX711::getAx128ChannelValue(uint16_t hx711_idx)
{
	Single_HX711 *_hx711 = &v_HX711[hx711_idx];
	return _hx711->storedReadings.ReadingAx128 - _hx711->offsetPerChannel.Ax128;
}

long Multiple_HX711::getAx64ChannelValue(uint16_t hx711_idx)
{
	Single_HX711 *_hx711 = &v_HX711[hx711_idx];
	return _hx711->storedReadings.ReadingAx64 - _hx711->offsetPerChannel.Ax64;
}

long Multiple_HX711::getBx32ChannelValue(uint16_t hx711_idx)
{
	Single_HX711 *_hx711 = &v_HX711[hx711_idx];
	return _hx711->storedReadings.ReadingBx32 - _hx711->offsetPerChannel.Bx32;
}

void Multiple_HX711::tare_Ax128(uint16_t hx711_idx)
{
	Single_HX711 *_hx711 = &v_HX711[hx711_idx];
	long _sum = 0;
	long sizeofvector = _hx711->historyStoredReadings.v_Ax128.size();
	for (uint8_t _idx = 0; _idx < sizeofvector; _idx++)
	{
		_sum += _hx711->historyStoredReadings.v_Ax128[_idx];
	}
	
	if (sizeofvector == 0)
	{
		Serial.println("tare_Ax128 DIVISION BY 0 !!!");
		return;
	}
	long _average = _sum / sizeofvector;
	set_offset_Ax128(hx711_idx, _average);
}

void Multiple_HX711::set_offset_Ax128(uint16_t hx711_idx, double offset)
{
	Single_HX711 *_hx711 = &v_HX711[hx711_idx];
	_hx711->offsetPerChannel.Ax128 = offset;
}

void Multiple_HX711::tare_Ax64(uint16_t hx711_idx)
{
	Single_HX711 *_hx711 = &v_HX711[hx711_idx];
	long _sum = 0;
	short sizeofvector = _hx711->historyStoredReadings.v_Ax64.size();
	for (uint8_t _idx = 0; _idx < sizeofvector; _idx++)
	{
		_sum += _hx711->historyStoredReadings.v_Ax64[_idx];
	}
	if (sizeofvector == 0)
	{
		Serial.println("tare_Ax64 DIVISION BY 0 !!!");
	}
	long _average = _sum / sizeofvector;
	set_offset_Ax64(hx711_idx, _average);
}

void Multiple_HX711::set_offset_Ax64(uint16_t hx711_idx, double offset)
{
	Single_HX711 *_hx711 = &v_HX711[hx711_idx];
	_hx711->offsetPerChannel.Ax64 = offset;
}

void Multiple_HX711::tare_Bx32(uint16_t hx711_idx)
{
	Single_HX711 *_hx711 = &v_HX711[hx711_idx];
	long _sum = 0;
	long sizeofvector = _hx711->historyStoredReadings.v_Bx32.size();
	for (uint8_t _idx = 0; _idx < sizeofvector; _idx++)
	{
		_sum += _hx711->historyStoredReadings.v_Bx32[_idx];
	}
	if (sizeofvector == 0)
	{
		Serial.println("tare_Bx32 DIVISION BY 0 !!!");
		return;
	}
	long _average = _sum / sizeofvector;
	set_offset_Bx32(hx711_idx, _average);
}

void Multiple_HX711::set_offset_Bx32(uint16_t hx711_idx, double offset)
{
	Single_HX711 *_hx711 = &v_HX711[hx711_idx];
	_hx711->offsetPerChannel.Bx32 = offset;
}

unsigned long Multiple_HX711::getLastElapsedTimeBetweenReadings()
{
	return lastElapsedMicros;
}

void Multiple_HX711::power_down()
{
	digitalWrite(PD_SCK, LOW);
	digitalWrite(PD_SCK, HIGH);
}

void Multiple_HX711::power_up()
{
	digitalWrite(PD_SCK, LOW);
}
