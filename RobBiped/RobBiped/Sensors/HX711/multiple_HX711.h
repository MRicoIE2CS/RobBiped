/*
 * multiple_HX711.h
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

#ifndef MULTIPLE_HX711_h
#define MULTIPLE_HX711_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <vector>
#include <array>
#include <string>
#include "../../Main/Configs.h"		// Dependant on Configuration::hx711_number

using std::vector;
using std::array;
using Configuration::hx711_number;

class Multiple_HX711
{
private:

	friend class ForceSensorsManager;

	uint8_t hx711_number_ = 0;

	byte PD_SCK_;	// Power Down and Serial Clock Input Pin (Common to all HX711)
	
	enum class Channel { Ax128, Bx32, Ax64 };
		
	struct CombinedOutputData { uint16_t hx711_idx; Channel _channel; int32_t _valueRead; };
	
	struct ActiveChannels {
		bool Ax128 = false;
		bool Ax64 = true;
		bool Bx32 = true;
	};
	struct OffsetPerChannel{
		int32_t Ax128;
		int32_t Ax64;
		int32_t Bx32;
	};
	struct StoredReadings {
		int32_t ReadingAx128;
		int32_t ReadingAx64;
		int32_t ReadingBx32;
	};
	struct LastStoredReadings{
		vector<int32_t> v_Ax128;
		vector<int32_t> v_Ax64;
		vector<int32_t> v_Bx32;
	};
	
	struct Single_HX711 
	{
		byte DIN;		// Serial Data Output Pin
		OffsetPerChannel offsetPerChannel;
		StoredReadings storedReadings;
		LastStoredReadings historyStoredReadings;
	};
	
	Channel channel_ = Channel::Ax64;
	ActiveChannels active_channels_;
	byte channel_selection_additional_bits_;		// Additional bits on dataframe for channel selection of next reading
	
	
	std::vector<Single_HX711> v_HX711_;	// Vector of HX711 structs
	
	uint32_t last_reading_micros_;
	uint32_t last_elapsed_micros_;
	
	Channel commute_next_channel(bool forceNextChannel = false, Channel _channel = Channel::Ax64);
	byte set_channel_selection_bits(Channel _nextChannel, bool forceNextSelection = false, short _sel = 3);
	
	void get_DIN_pins_array(byte *_array);
	void read_shiftIn(uint8_t clockPin, byte *DIN_array, bool _readings[hx711_number][8]);
	void construct_read(uint8_t idx_byte_sel, uint8_t clockPin, uint8_t bitOrder, byte _arr_data[hx711_number][3], bool _readings[hx711_number][8]);
	
	// Obtains a reading, and sets channel for next one
	void read_and_commute_next_channel();
	
	uint16_t historyLength = 10;
	void history_append(uint16_t hx711_idx, Channel channel, int32_t _reading);

 public:
	
	// Define clock and data pin
	Multiple_HX711();

	virtual ~Multiple_HX711();
	
	void configure(byte _DINs[], byte pd_sck);

	// Check if HX711s are ready
	// from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
	// input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
	bool are_xh711_ready();
	
	// Read if available and update
	bool update();
	
	// Set active channels
	void set_sctive_channels(bool _Ax128, bool _Ax64, bool _Bx32);
	
	
	// Get each channel's value
	int32_t get_Ax128_channel_value(uint16_t hx711_idx);
	int32_t get_Ax64_channel_value(uint16_t hx711_idx);
	int32_t get_Bx32_channel_value(uint16_t hx711_idx);
	
	// Tare: set the OFFSET value for tare weight; times = how many times to read the tare value
	void tare_Ax128(uint16_t hx711_idx);
	void set_offset_Ax128(uint16_t hx711_idx, double offset);
	void tare_Ax64(uint16_t hx711_idx);
	void set_offset_Ax64(uint16_t hx711_idx, double offset);
	void tare_Bx32(uint16_t hx711_idx);
	void set_offset_Bx32(uint16_t hx711_idx, double offset);
	
	// Get last elapsed time between readings
	uint32_t get_last_elapsed_time_between_readings();

	// Puts the chip into power down mode
	void power_down();

	// Wakes up the chip after power down mode
	void power_up();
};

#endif /* MULTIPLE_HX711_h */

