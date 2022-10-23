
/*
 * multiple_HX711.h
 *
 * Created: 20/03/2022
 * Author: MRICO
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
	uint8_t _hx711_number = 0;

	byte PD_SCK;	// Power Down and Serial Clock Input Pin (Common to all HX711)
	
	enum class Channel { Ax128, Bx32, Ax64 };
		
	struct CombinedOutputData { uint16_t hx711_idx; Channel _channel; long _valueRead; };
	
	struct ActiveChannels {
		bool Ax128 = false;
		bool Ax64 = true;
		bool Bx32 = true;
	};
	struct OffsetPerChannel{
		long Ax128;
		long Ax64;
		long Bx32;
	};
	struct StoredReadings {
		long ReadingAx128;
		long ReadingAx64;
		long ReadingBx32;
	};
	struct LastStoredReadings{
		vector<long> v_Ax128;
		vector<long> v_Ax64;
		vector<long> v_Bx32;
	};
	
	struct Single_HX711 
	{
		byte DIN;		// Serial Data Output Pin
		OffsetPerChannel offsetPerChannel;
		StoredReadings storedReadings;
		LastStoredReadings historyStoredReadings;
	};
	
	Channel channel = Channel::Ax64;
	ActiveChannels activeChannels;
	byte ChSelBits;		// Additional bits on dataframe for channel selection of next reading
	
	
	std::vector<Single_HX711> v_HX711;	// Vector of HX711 structs
	
	unsigned long lastReadingMicros;
	unsigned long lastElapsedMicros;
	
	Channel commuteNextChannel(bool forceNextChannel = false, Channel _channel = Channel::Ax64);
	byte setChannelSelectionBits(Channel _nextChannel, bool forceNextSelection = false, short _sel = 3);
	
	void get_DIN_pins_array(byte *_array);
	void read_shiftIn(uint8_t clockPin, byte *DIN_array, bool _readings[hx711_number][8]);
	void construct_read(uint8_t idx_byte_sel, uint8_t clockPin, uint8_t bitOrder, byte _arr_data[hx711_number][3], bool _readings[hx711_number][8]);
	
	// Obtains a reading, and sets channel for next one
	void readAndCommuteNextChannel();
	
	uint16_t historyLength = 10;
	void historyAppend(uint16_t hx711_idx, Channel channel, long _reading);

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
	void setActiveChannels(bool _Ax128, bool _Ax64, bool _Bx32);
	
	
	// Get each channel's value
	long getAx128ChannelValue(uint16_t hx711_idx);
	long getAx64ChannelValue(uint16_t hx711_idx);
	long getBx32ChannelValue(uint16_t hx711_idx);
	
	// Tare: set the OFFSET value for tare weight; times = how many times to read the tare value
	void tare_Ax128(uint16_t hx711_idx);
	void set_offset_Ax128(uint16_t hx711_idx, double offset);
	void tare_Ax64(uint16_t hx711_idx);
	void set_offset_Ax64(uint16_t hx711_idx, double offset);
	void tare_Bx32(uint16_t hx711_idx);
	void set_offset_Bx32(uint16_t hx711_idx, double offset);
	
	// Get last elapsed time between readings
	unsigned long getLastElapsedTimeBetweenReadings();

	// Puts the chip into power down mode
	void power_down();

	// Wakes up the chip after power down mode
	void power_up();
};

#endif /* MULTIPLE_HX711_h */

