#pragma once

#include <Arduino.h>
#include <inttypes.h>

#define ANALOG_OUTPUT OUTPUT
#define ANALOG_INPUT  INPUT

// #define ADP_DIV2      0b000
#define ADPS_DIV2      0b001
#define ADPS_DIV4      0b010
#define ADPS_DIV8      0b011
#define ADPS_DIV16     0b100
#define ADPS_DIV32     0b101
#define ADPS_DIV64     0b110
#define ADPS_DIV128    0b111

class FastIO {
private:
	uint8_t pin_num;

	/**
	 * prevent from invalid memory access (for pointer access)
	 */
	uint8_t __invalid8t;

	/**
	 * for digital IO
	 */
	uint8_t pinmask;
	uint8_t pinmask_; // = ~pinmask
	volatile uint8_t *inport;
	volatile uint8_t *outport;

	/**
	 * for analog output
	 */
	volatile uint8_t *pwm_OCR;
	volatile uint8_t *pwm_TCR;
	uint8_t pwm_on_mask;
	uint8_t pwm_off_mask;
	bool not_on_timer;

	/**
	 * for analog input
	 */
	uint8_t analog_reference = DEFAULT;
	uint8_t adc_channel;
	uint8_t adps_value; // ADC prescaler value (2 ~ 128)
	
public:
	/**
	 * Default constructor.
	 * Configure nothing and work nothing.
	 */
	FastIO() {
		// protect from invalid memory access
		inport  = &__invalid8t;
		outport = &__invalid8t;
		pwm_TCR = &__invalid8t;
		pwm_OCR = &__invalid8t;
	}
	
	/**
	 * Constructor.
	 * Configures the specified pin to behave either as an input or an output.
	 * This function configures same as the pinMode(pin, mode) of Arduino API.
	 * @param pin  The pin number
	 * @param mode The pin begavior (INPUT, OUTPUT, or INPUT_PULLUP)
	 */
	FastIO(uint8_t pin, uint8_t mode) {
		pin_num = pin;
	
		// protect from invalid memory access
		inport  = &__invalid8t;
		outport = &__invalid8t;
		pwm_TCR = &__invalid8t;
		pwm_OCR = &__invalid8t;
		if ( pin == NOT_A_PIN || ( mode != OUTPUT && mode != INPUT && mode != INPUT_PULLUP && mode != ANALOG_OUTPUT && mode != ANALOG_INPUT ) ) {
			// arguments error
			return;
		}
	
		// set the pin IO with Arduino API
		if ( mode != ANALOG_INPUT ) {
			pinMode(pin, mode);
		}
	
		// for digital IO
		uint8_t port;
		port     = digitalPinToPort(pin);
		pinmask  = digitalPinToBitMask(pin);
		pinmask_ = ~pinmask;
		inport   = portInputRegister(port);
		outport  = portOutputRegister(port);
	
		// for analog output
		uint8_t timer;
		timer        = digitalPinToTimer(pin);
		not_on_timer = false;
		switch ( timer ) {
			#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
				case TIMER0A: pwm_TCR = &TCCR0;  pwm_OCR = (uint8_t*)&OCR0;  pwm_on_mask = _BV(COM00);  break;
			#endif
			#if defined(TCCR0A) && defined(COM0A1)
				case TIMER0A: pwm_TCR = &TCCR0A; pwm_OCR = (uint8_t*)&OCR0A; pwm_on_mask = _BV(COM0A1); break;
			#endif
			#if defined(TCCR0A) && defined(COM0B1)
				case TIMER0B: pwm_TCR = &TCCR0A; pwm_OCR = (uint8_t*)&OCR0B; pwm_on_mask = _BV(COM0B1); break;
			#endif
			#if defined(TCCR1A) && defined(COM1A1)
				case TIMER1A: pwm_TCR = &TCCR1A; pwm_OCR = (uint8_t*)&OCR1A; pwm_on_mask = _BV(COM1A1); break;
			#endif
			#if defined(TCCR1A) && defined(COM1B1)
				case TIMER1B: pwm_TCR = &TCCR1A; pwm_OCR = (uint8_t*)&OCR1B; pwm_on_mask = _BV(COM1B1); break;
			#endif
			#if defined(TCCR1A) && defined(COM1C1)
				case TIMER1C: pwm_TCR = &TCCR1A; pwm_OCR = (uint8_t*)&OCR1C; pwm_on_mask = _BV(COM1C1); break;
			#endif
			#if defined(TCCR2) && defined(COM21)
				case TIMER2:  pwm_TCR = &TCCR2;  pwm_OCR = (uint8_t*)&OCR2;  pwm_on_mask = _BV(COM21);  break;
			#endif
			#if defined(TCCR2A) && defined(COM2A1)
				case TIMER2A: pwm_TCR = &TCCR2A; pwm_OCR = (uint8_t*)&OCR2A; pwm_on_mask = _BV(COM2A1); break;
			#endif
			#if defined(TCCR2A) && defined(COM2B1)
				case TIMER2B: pwm_TCR = &TCCR2A; pwm_OCR = (uint8_t*)&OCR2B; pwm_on_mask = _BV(COM2B1); break;
			#endif
			#if defined(TCCR3A) && defined(COM3A1)
				case TIMER3A: pwm_TCR = &TCCR3A; pwm_OCR = (uint8_t*)&OCR3A; pwm_on_mask = _BV(COM3A1); break;
			#endif
			#if defined(TCCR3A) && defined(COM3B1)
				case TIMER3B: pwm_TCR = &TCCR3A; pwm_OCR = (uint8_t*)&OCR3B; pwm_on_mask = _BV(COM3B1); break;
			#endif
			#if defined(TCCR3A) && defined(COM3C1)
				case TIMER3C: pwm_TCR = &TCCR3A; pwm_OCR = (uint8_t*)&OCR3C; pwm_on_mask = _BV(COM3C1); break;
			#endif
			#if defined(TCCR4A) && !defined(COM4A0)
				case TIMER4A: pwm_TCR = &TCCR4A; pwm_OCR = (uint8_t*)&OCR4A; pwm_on_mask = _BV(COM4A1); break;
			#endif
	
			// only used on 32U4
			#if defined(TCCR4A) &&  defined(COM4A0)
				case TIMER4A: pwm_TCR = &TCCR4A; pwm_OCR = (uint8_t*)&OCR4A; pwm_on_mask = _BV(COM4A1); TCCR4A &= ~_BV(COM4A0); break;
			#endif
			
			#if defined(TCCR4A) && defined(COM4B1)
				case TIMER4B: pwm_TCR = &TCCR4A; pwm_OCR = (uint8_t*)&OCR4B; pwm_on_mask = _BV(COM4B1); break;
			#endif
			#if defined(TCCR4A) && defined(COM4C1)
				case TIMER4C: pwm_TCR = &TCCR4A; pwm_OCR = (uint8_t*)&OCR4C; pwm_on_mask = _BV(COM4C1); break;
			#endif
			#if defined(TCCR4C) && defined(COM4D1) && !defined(COM4D0)
				case TIMER4D: pwm_TCR = &TCCR4C; pwm_OCR = (uint8_t*)&OCR4D; pwm_on_mask = _BV(COM4D1); break;
			#endif
	
			// only used on 32U4
			#if defined(TCCR4C) && defined(COM4D1) &&  defined(COM4D0)
				case TIMER4D: pwm_TCR = &TCCR4C; pwm_OCR = (uint8_t*)&OCR4D; pwm_on_mask = _BV(COM4D1); TCCR4C &= ~_BV(COM4D0); break;
			#endif
							
			#if defined(TCCR5A) && defined(COM5A1)
				case TIMER5A: pwm_TCR = &TCCR5A; pwm_OCR = (uint8_t*)&OCR5A; pwm_on_mask = _BV(COM5A1); break;
			#endif
			#if defined(TCCR5A) && defined(COM5B1)
				case TIMER5B: pwm_TCR = &TCCR5A; pwm_OCR = (uint8_t*)&OCR5B; pwm_on_mask = _BV(COM5B1); break;
			#endif
			#if defined(TCCR5A) && defined(COM5C1)
				case TIMER5C: pwm_TCR = &TCCR5A; pwm_OCR = (uint8_t*)&OCR5C; pwm_on_mask = _BV(COM5C1); break;
			#endif
	
			case NOT_ON_TIMER:
			default:
				not_on_timer = true;
		}
		pwm_off_mask = ~pwm_on_mask;
	
		// for analog input
		#if defined(analogPinToChannel)
			#if defined(__AVR_ATmega32U4__)
				if (adc_channel >= 18) adc_channel -= 18;
			#endif
			adc_channel = analogPinToChannel(adc_channel);
		#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
			if (adc_channel >= 54) adc_channel -= 54;
		#elif defined(__AVR_ATmega32U4__)
			if (adc_channel >= 18) adc_channel -= 18;
		#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
			if (adc_channel >= 24) adc_channel -= 24;
		#else
			if (adc_channel >= 14) adc_channel -= 14;
		#endif

		// set ADC prescaler value
		adps_value = ADPS_DIV128 & 0b00000111;
	}

	/**
	 * Reads the value (HIGH or LOW) from the digital pin specified at the constructor.
	 * @return The value of the pin (HIGH or LOW)
	 */
	int digitalRead() {
		if ( read() ) return HIGH;
		else return LOW;
	}

	/**
	 * Writes the value (HIGH or LOW) to the digital pin specified at the constructor.
	 * @param val The value write to the pin (HIGH or LOW)
	 */
	void digitalWrite(uint8_t val){
		if ( val == LOW ) { writeLow(); }
		else              { writeHigh(); }
	}

	/**
	 * Reads the value from the analog pin specified at the constructor.
	 * @return The value ov the pin (0 to 1023)
	 */
	int16_t analogRead() {
		uint8_t low;
		uint16_t high;
		uint16_t ret = 0;
		
		// ADC additional channel select
		#if defined(ADCSRB) && defined(MUX5)
			ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((adc_channel >> 3) & 0x01 ) << MUX5);
		#endif
		  
		// set analog reference, alignment(right), channel
		#if defined(ADMUX)
			ADMUX = (analog_reference << 6) | (adc_channel & 0x07);
		#endif
		
		#if defined(ADCSRA) && defined(ADCL)
			// set ADC Prescaler value
			ADCSRA = (0b11111000 & ADCSRA) | adps_value;
			// start the conversion
			ADCSRA |= _BV(ADSC);		
			// wait until the conversion finishes
			while ( bit_is_set(ADCSRA, ADSC) ) { }
			// read value
			// note that must read ADCL first
			low  = ADCL;
			high = ADCH;
			high <<= 8;
			// combine the two bytes
			ret += high | low;
		#else
			// we dont have an ADC, return 0
			ret = 0;
		#endif
		
		return ret;
	}

	/**
	 * Writes an analog value to a pin as PWM wave.
	 * @param val the duty cycle (between 0 and 255)
	 */
	void analogWrite(int val) {
		if (not_on_timer) {
			digitalWrite((val < 128) ? LOW : HIGH);
		} else {
			if ( val <= 0 ) {
				writeLow();
			} else if ( val >= 255 ) {
				writeHigh();
			} else {
				// turn on pwm
				*pwm_TCR |= pwm_on_mask;
				// set pwm duty
				*pwm_OCR  = val;
			}
		}
	}

	/**
	 * Read the value from the digital pin specified at the constructor.
	 * Return 0 in the case of the pin is LOW, but return bit mask in the case of other than it.
	 * For example, return 0b00000010 if the digital pin 9 of Arduino Uno is HIGH, return 0 if the pin is LOW.
	 * @return The value of the masked input port register
	 */
	uint8_t read() {
		return *inport & pinmask;
	}

	/**
	 * Writes the HIGH to the digital pin specified at the constructor.
	 */
	void writeHigh() {
		// turn off pwm
		*pwm_TCR &= pwm_off_mask;
		// set a bit for the specified pin
		*outport |= pinmask;
	}

	/**
	 * Writes the LOW to the digital pin specified at the constructor.
	 */
	void writeLow() {
		// turn off pwm
		*pwm_TCR &= pwm_off_mask;
		// clear a bit for the specified pin
		*outport &= pinmask_;
	}

	void analogReference(uint8_t mode) {
		analog_reference = mode;
	}

	uint8_t getPinNumber() {
		return pin_num;
	}

	/**
	 * Sets the ADC prescaler select bits (0b001 ~ 0b111, ADPS_DIV2 ~ ADPS_DIV128)
	 * @param adps ADPS bits
	 */
	void setADPS(uint8_t adps) {
		adps_value = adps & 0b00000111;
	}
};
