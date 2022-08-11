#ifndef HX711_h
#define HX711_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class HX711
{
	private:
	byte PD_SCK;	// Power Down and Serial Clock Input Pin
	byte DOUT;		// Serial Data Output Pin
	enum class Channel { Ax128, Bx32, Ax64 };
	Channel channel = Channel::Ax128;
	struct CombinedOutputData { Channel _channel; long _valueRead; };
	struct ActiveChannels {
		bool Ax128 = false;
		bool Ax64 = true;
		bool Bx32 = true;
	}activeChannels;
	struct OffsetPerChannel{
		long Ax128;
		long Ax64;
		long Bx32;
	}offsetPerChannel;
	struct StoredReadings {
		long ReadingAx128;
		long ReadingAx64;
		long ReadingBx32;
	}storedReadings;
	byte ChSelBits;		// Additional bits on dataframe for channel selection of next reading
	byte GAIN;		// amplification factor
	long OFFSET;	// used for tare weight
	float SCALE;	// used to return weight in grams, kg, ounces, whatever
	
	unsigned long lastReadingMicros;
	unsigned long lastElapsedMicros;

	public:
	
	
	
	// define clock and data pin, channel, and gain factor
	// channel selection is made by passing the appropriate gain: 128 or 64 for channel A, 32 for channel B
	// gain: 128 or 64 for channel A; channel B works with 32 gain factor only
	HX711(byte dout, byte pd_sck, byte gain = 128);

	virtual ~HX711();

	// check if HX711 is ready
	// from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
	// input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
	bool is_ready();

	// set the gain factor; takes effect only after a call to read()
	// channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
	// depending on the parameter, the channel is also set to either A or B
	void set_gain(byte gain = 128);

	// waits for the chip to be ready and returns a reading
	long read();
	
	//NEW: read if available and update
	void update();
	
	//NEW: Set active channels
	void setActiveChannels(bool _Ax128, bool _Ax64, bool _Bx32);
	
	// NEW: doesn't wait for the chip to be ready, returns a reading, and sets channel for next one
	CombinedOutputData readAndConmuteNextChannel();
	Channel conmuteNextChannel(bool forceNextChannel = false, Channel _channel = Channel::Ax128);
	byte setChannelSelectionBits(Channel _nextChannel, bool forceNextSelection = false, short _sel =1);
	
	// NEW: Get each channel's value
	long getAx128ChannelValue();
	long getAx64ChannelValue();
	long getBx32ChannelValue();
	
	// NEW TARE: set the OFFSET value for tare weight; times = how many times to read the tare value
	void tare_Ax128(byte times = 10);
	void set_offset_Ax128(double offset);
	void tare_Ax64(byte times = 10);
	void set_offset_Ax64(double offset);
	void tare_Bx32(byte times = 10);
	void set_offset_Bx32(double offset);
	
	// NEW: Get last elapsed time between readings
	unsigned long getLastElapsedTimeBetweenReadings();

	// returns an average reading; times = how many times to read
	long read_average(byte times = 10);

	// returns (read_average() - OFFSET), that is the current value without the tare weight; times = how many readings to do
	double get_value(byte times = 1);

	// returns get_value() divided by SCALE, that is the raw value divided by a value obtained via calibration
	// times = how many readings to do
	float get_units(byte times = 1);

	// set the OFFSET value for tare weight; times = how many times to read the tare value
	void tare(byte times = 10);

	// set the SCALE value; this value is used to convert the raw data to "human readable" data (measure units)
	void set_scale(float scale = 1.f);

	// get the current SCALE
	float get_scale();

	// set OFFSET, the value that's subtracted from the actual reading (tare weight)
	void set_offset(long offset = 0);

	// get the current OFFSET
	long get_offset();

	// puts the chip into power down mode
	void power_down();

	// wakes up the chip after power down mode
	void power_up();
};

#endif /* HX711_h */

