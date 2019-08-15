#ifndef PC_INTERRUPT_H
#define PC_INTERRUPT_H

#include <Arduino.h>
#include "FastIO.h"

class PinChangeInterrupt {
public:
	/**
	 * Arrays for storing the callback functions for each PCINT interrupt.
	 */
	void (*pcint0_funcs[8]) () = {};
	void (*pcint1_funcs[8]) () = {};
	void (*pcint2_funcs[8]) () = {};
	/**
	 * The FastIO arrays for each PCINT pin.
	 * It will be used only to read a PCINT pin value.
	 */
	FastIO *pcint0_IO[8];
	FastIO *pcint1_IO[8];
	FastIO *pcint2_IO[8];
	/**
	 * Arrays for storing a previous PCINT pin value.
	 */
	volatile uint8_t pcint0_prev[8];
	volatile uint8_t pcint1_prev[8];
	volatile uint8_t pcint2_prev[8];

	/**
	 * Default constructor.
	 * Do nothing.
	 */
	PinChangeInterrupt() {};

	/**
	 * Set callback fanction called when specified pin changed and enable the pin change interrupt.
	 * ! NOTICE !
	 *     The first parameter of attachInterrupt(interrupt, func, mode) of Arduino API is an "interrupt number" (not a "pin number").
	 *     But it of this function is a "pin number".
	 * @param pin      The pin number
	 * @param callback The function to call when interrupt occurs
	 */
	void attachInterrupt(uint8_t pin, void (*callback)(void)) {
		// get the the PCINT pins/ports corresponding to the specified pin
		volatile uint8_t *pcicr_reg = digitalPinToPCICR(pin);
		volatile uint8_t *pcmsk_reg = digitalPinToPCMSK(pin);
		uint8_t pcicr_bit = digitalPinToPCICRbit(pin);
		uint8_t pcmsk_bit = digitalPinToPCMSKbit(pin);

		if ( pcicr_reg == 0 ) {
			// specified pin has no PCINT function
			return;
		}

		// set the corresponding bits to enable the pin change interrupt
		*pcicr_reg |= _BV(pcicr_bit);
		*pcmsk_reg |= _BV(pcmsk_bit);

		FastIO *io = new FastIO(pin, INPUT);
		int val = io->read();
		if ( pcicr_bit == 0 ) {
			// PCINT0
			pcint0_funcs[pcmsk_bit] = callback;
			pcint0_IO[pcmsk_bit]    = io;
			pcint0_prev[pcmsk_bit]  = val;
		} else if ( pcicr_bit == 1 ) {
			// PCINT1
			pcint1_funcs[pcmsk_bit] = callback;
			pcint1_IO[pcmsk_bit]    = io;
			pcint1_prev[pcmsk_bit]  = val;
		} else if ( pcicr_bit == 2 ) {
			// PCINT2
			pcint2_funcs[pcmsk_bit] = callback;
			pcint2_IO[pcmsk_bit]    = io;
			pcint2_prev[pcmsk_bit]  = val;
		}

		// enable interrupt
		sei();
	}

	/**
	 * Detach the pin change interrupt.
	 * @param pin The pin number
	 */
	void detachInterrupt(uint8_t pin) {
		// get the the PCINT pins/ports corresponding to the specified pin
		volatile uint8_t *pcicr_reg = digitalPinToPCICR(pin);
		volatile uint8_t *pcmsk_reg = digitalPinToPCMSK(pin);
		uint8_t pcicr_bit = digitalPinToPCICRbit(pin);
		uint8_t pcmsk_bit = digitalPinToPCMSKbit(pin);

		if ( pcicr_reg == 0 ) {
			// specified pin has no PCINT function
			return;
		}

		// disable the PCMSK bit corresponding to the specified pin
		*pcmsk_reg &= ~_BV(pcmsk_bit);
		if ( *pcmsk_reg == 0 ) {
			// clear the corresponding PCICR bit if all the PCMSK bit is cleared
			*pcicr_reg &= ~_BV(pcicr_bit);
		}

		if ( pcicr_bit == 0 ) {
			// PCINT0
			pcint0_funcs[pcmsk_bit] = 0;
		} else if ( pcicr_bit == 1 ) {
			// PCINT1
			pcint1_funcs[pcmsk_bit] = 0;
		} else if ( pcicr_bit == 2 ) {
			// PCINT2
			pcint2_funcs[pcmsk_bit] = 0;
		}
	}
};

PinChangeInterrupt PCInterrupt;

ISR(PCINT0_vect) {
	bool changeFlag;
	uint8_t val;

	while ( 1 ) {
		changeFlag = false;

		for ( uint8_t i=0; i<8; ++i ) {
			if ( PCInterrupt.pcint0_funcs[i] == 0 ) { continue; }
			val = PCInterrupt.pcint0_IO[i]->read();
			if (PCInterrupt.pcint0_prev[i] == val ) { continue; }
			PCInterrupt.pcint0_funcs[i]();
			PCInterrupt.pcint0_prev[i] = val;
			changeFlag = true;
		}

		if ( !changeFlag ) break;
	}
}

ISR(PCINT1_vect) {
	bool changeFlag;
	uint8_t val;

	while ( 1 ) {
		changeFlag = false;

		for ( uint8_t i=0; i<8; ++i ) {
			if ( PCInterrupt.pcint1_funcs[i] == 0 ) { continue; }
			val = PCInterrupt.pcint1_IO[i]->read();
			if (PCInterrupt.pcint1_prev[i] == val ) { continue; }
			PCInterrupt.pcint1_funcs[i]();
			PCInterrupt.pcint1_prev[i] = val;
			changeFlag = true;
		}

		if ( !changeFlag ) break;
	}
}

ISR(PCINT2_vect) {
	bool changeFlag;
	uint8_t val;

	while ( 1 ) {
		changeFlag = false;

		for ( uint8_t i=0; i<8; ++i ) {
			if ( PCInterrupt.pcint2_funcs[i] == 0 ) { continue; }
			val = PCInterrupt.pcint2_IO[i]->read();
			if (PCInterrupt.pcint2_prev[i] == val ) { continue; }
			PCInterrupt.pcint2_funcs[i]();
			PCInterrupt.pcint2_prev[i] = val;
			changeFlag = true;
		}

		if ( !changeFlag ) break;
	}
}

#endif /* PC_INTERRUPT_H */