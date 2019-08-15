#ifndef ALPA_CAR_H
#define ALPA_CAR_H

#include <inttypes.h>
#include <MsTimer2.h>
#include "FastIO.h"
#include "PCInterrupt.h"
#include "PIDController.h"
#include "PosEstimator.h"

// Pin Definitions
// Motor 1 pin configuration
#define PWM_R      9
#define DIR_R      7
// Motor 2 pin configuration
#define PWM_L      10
#define DIR_L      8
// Rotary Encoder pin configuration
#define R_A        SCK
#define R_B        MOSI
#define L_A        MISO
#define L_B        4
// User IO pin configuration
#define SW1_PIN    5
#define SW2_PIN    6

// prototype declaration
void intr_R_cnt();
void intr_L_cnt();
void intr_timer2();

class AlpaCarShield {
private:
	/**
	 * Rotary encoder counter value (for count up)
	 */
	volatile int16_t cnt_R = 0;
	volatile int16_t cnt_L = 0;

	/**
	 * user interface IO
	 */
	FastIO *sw1;
	FastIO *sw2;
	/**
	 * motor IO
	 */
	FastIO *mot_R_pwm;
	FastIO *mot_R_dir;
	FastIO *mot_L_pwm;
	FastIO *mot_L_dir;
	/**
	 * rotary encoder IO
	 */
	FastIO *rot_R_A;
	FastIO *rot_R_B;
	FastIO *rot_L_A;
	FastIO *rot_L_B;

	PIController<double> *pid_R;
	PIController<double> *pid_L;

public:
	// for interrupt test
	FastIO *testpin;
	uint8_t testpin_flag = 0;
	void teston() {
		if ( this->testpin_flag == 0 ) { return; }
		this->testpin->writeHigh();
	}
	void testoff() {
		if ( this->testpin_flag == 0 ) { return; }
		this->testpin->writeLow();
	}

	/**
	 * Latest rotary encoder counter value (for representing speed)
	 */
	volatile int16_t spd_R = 0;
	volatile int16_t spd_L = 0;
	/**
	 * Target speed
	 */
	volatile int16_t trg_R = 0;
	volatile int16_t trg_L = 0;

	/**
	 * Incremented when Timer2 interrupt is occurred
	 */
	volatile int8_t t2_updated = 0;

	AlpaCarShield() {
		// IO settings
		this->sw1       = new FastIO(SW1_PIN, INPUT);
		this->sw2       = new FastIO(SW2_PIN, INPUT);
		this->mot_R_pwm = new FastIO(PWM_R,   OUTPUT);
		this->mot_R_dir = new FastIO(DIR_R,   OUTPUT);
		this->mot_L_pwm = new FastIO(PWM_L,   OUTPUT);
		this->mot_L_dir = new FastIO(DIR_L,   OUTPUT);
		this->rot_R_A   = new FastIO(R_A,     INPUT);
		this->rot_R_B   = new FastIO(R_B,     INPUT);
		this->rot_L_A   = new FastIO(L_A,     INPUT);
		this->rot_L_B   = new FastIO(L_B,     INPUT);

		// initialize pin output
		this->mot_R_pwm->digitalWrite(LOW);
		this->mot_R_dir->digitalWrite(LOW);
		this->mot_L_pwm->digitalWrite(LOW);
		this->mot_L_dir->digitalWrite(LOW);

		// initialize PID controller
		this->pid_R = new PIController<double>(0, 0);
		this->pid_L = new PIController<double>(0, 0);
	}

	void set_PID_params(double kp, double ki, double integral_max) {
		this->pid_R->set_params(kp, ki);
		this->pid_L->set_params(kp, ki);
		this->pid_R->set_limit(integral_max);
		this->pid_L->set_limit(integral_max);
	}
	void set_PID_params(double kp, double ki) {
		this->pid_R->set_params(kp, ki);
		this->pid_L->set_params(kp, ki);
	}

	void set_target_count(int r, int l) {
		this->trg_R = r;
		this->trg_L = l;
	}

	void enable_testpin(uint8_t pin) {
		this->testpin = new FastIO(pin, OUTPUT);
		this->testpin_flag = 1;
	}
	void disable_testpin() {
		this->testpin_flag = 0;
		this->testpin = new FastIO(this->testpin->getPinNumber(), INPUT);
	}

	int read_switch1() {
		return this->sw1->digitalRead();
	}
	int read_switch2() {
		return this->sw2->digitalRead();
	}

	void start(unsigned long interval_ms = 25) {
		// rotary encoder interrupt settings
		PCInterrupt.attachInterrupt(R_A, intr_R_cnt);
		PCInterrupt.attachInterrupt(L_A, intr_L_cnt);
		// initialize vars
		this->cnt_R = 0;
		this->cnt_L = 0;
		this->spd_R = 0;
		this->spd_L = 0;

		// enable timer2 interrupt
		MsTimer2::set(interval_ms, intr_timer2);
		MsTimer2::start();
	}

	void stop() {
		PCInterrupt.detachInterrupt(R_A);
		PCInterrupt.detachInterrupt(L_A);
	}

	void __set_motor_pwm(int r, int l) {
		uint8_t dir_r = ( r < 0 ) ? ( HIGH ) : ( LOW );
		uint8_t dir_l = ( l < 0 ) ? ( HIGH ) : ( LOW );

		r = abs(r);
		l = abs(l);

		if ( r > 255 ) { r = 255; }
		if ( l > 255 ) { l = 255; }

		this->mot_R_dir->digitalWrite(dir_r);
		this->mot_L_dir->digitalWrite(dir_l);
		this->mot_R_pwm->analogWrite(r);
		this->mot_L_pwm->analogWrite(l);
	}

	void __pid_control() {
		double pid_R_value = this->pid_R->control(spd_R, trg_R);
		double pid_L_value = this->pid_R->control(spd_L, trg_L);
		this->__set_motor_pwm(pid_R_value, pid_L_value);
	}


	//**************************************************************
	// Interrupt Functions
	//**************************************************************
	void __intr_R_cnt_sub() {
		uint8_t a = this->rot_R_A->read();
		uint8_t b = this->rot_R_B->read();

		if ( ( a == 0 && b == 0 ) || ( a != 0 && b != 0 ) ) {
			++ this->cnt_R;
		} else {
			-- this->cnt_R;
		}
	}
	void __intr_L_cnt_sub() {
		uint8_t a = this->rot_L_A->read();
		uint8_t b = this->rot_L_B->read();

		if ( ( a == 0 && b == 0 ) || ( a != 0 && b != 0 ) ) {
			++ this->cnt_L;
		} else {
			-- this->cnt_L;
		}
	}
	void __save_speed() {
		this->spd_R = this->cnt_R;
		this->spd_L = this->cnt_L;
		this->cnt_R = 0;
		this->cnt_L = 0;
	}

};
AlpaCarShield AlpaCar;

//**************************************************************
// Interrupt Functions
//**************************************************************
/**
 * Pin change interrupt callback function.
 * This function called when ROT_1 pin value changes.
 */
void intr_R_cnt() {
	AlpaCar.__intr_R_cnt_sub();
}

/**
 * Pin change interrupt callback function.
 * This function called when ROT_1 pin value changes.
 */
void intr_L_cnt() {
	AlpaCar.__intr_L_cnt_sub();
}

/**
 * Timer2 overflow interrupt callback function by MsTimer2 library.
 * In this function, do the followings:
 *     2. Store rotary encoder counter and reset.
 *     2. Motor PID controll.
 *     3. (Update car position from rotx_cnt)
 *     4. Callback the function specified at attachInterrupt()
 *     5. Notify Timer2 interrupt occurred
 */
void intr_timer2() {
	AlpaCar.teston();

	/************** 1 **************/
	// store current speed
	AlpaCar.__save_speed();

	/************** 2 **************/
	// Motor PID controll.
	AlpaCar.__pid_control();

	/************** 3 **************/
	// Update car position from rotx_cnt
	// AlpaCar.__update_pos();

	/************** 4 **************/
	// Callback the function specified at attachInterrupt()
	// AlpaCar.__callAttachedFunction();

	/************** 5 **************/
	// Notify Timer2 interrupt occurred
	++ AlpaCar.t2_updated;

	AlpaCar.testoff();
}


#endif /* ALPA_CAR_H */
