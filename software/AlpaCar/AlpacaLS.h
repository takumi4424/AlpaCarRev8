#ifndef ALPACA_LINE_SENSOR_H
#define ALPACA_LINE_SENSOR_H

#include <inttypes.h>
#include <Wire.h>

#define STATE_NOLINE 0x00
#define STATE_INLINE 0x01
#define STATE_CALIB  0x02

class AlpacaLS {
private:
	int address;

public:
	uint8_t state;
	double position;
	bool is_shade[12];
	uint8_t ratio[12];

	AlpacaLS(uint8_t addr = 0x38) {
		Wire.begin();
		address = addr;
	}

	bool communicate() {
		/**
		* 0:
		*     state
		* 1~2:
		*     position(int16_t, ×100)
		* 3~4:
		*     is_shade(int16_t, bit)
		* 5~16:
		*     ratio(uint8_t, ×255)
		*/

		uint8_t data[17];
		int datanum;
		int16_t buf;

		datanum = Wire.requestFrom(address, 17);

		if ( datanum != 17 ) { return false; }

		for ( uint8_t i=0; i<17; ++i ) { data[i] = Wire.read(); }

		if ( data[0] != STATE_NOLINE && data[0] != STATE_INLINE && data[0] != STATE_CALIB ) { Serial.println(data[0]); return false; }

		state = data[0];

		buf =   data[2];
		buf <<= 8;
		buf +=  data[1];
		position = 0.01 * buf;

		buf =   data[4];
		buf <<= 8;
		buf +=  data[3];

		for ( uint8_t i=0; i<12; ++i ) {
			is_shade[i] = (buf & (1 << i)) != 0;
		}

		for ( uint8_t i=0; i<12; ++i ) {
			ratio[i] = data[i+5];
		}

		return true;
	}
};

#endif /* ALPACA_LINE_SENSOR_H */