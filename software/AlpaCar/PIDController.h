#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <inttypes.h>

template<class pidint_t = double>
class PIController {
private:
	pidint_t kp;
	pidint_t ki;

	uint8_t has_limit = 0;
	pidint_t integral_max = 0;
public:
	PIController(pidint_t kp_value, pidint_t ki_value) {
		this->set_params(kp_value, ki_value);
	}
	PIController(pidint_t kp_value, pidint_t ki_value, pidint_t limit) {
		this->set_params(kp_value, ki_value);
		this->set_limit(limit);
	}

	void set_params(pidint_t kp_value, pidint_t ki_value) {
		this->kp = kp_value;
		this->ki = ki_value;
	}

	void set_limit(pidint_t limit) {
		this->has_limit = 1;
		this->integral_max = limit;
	}

	void unset_limit() {
		this->has_limit = 0;
	}

	pidint_t control(pidint_t now, pidint_t trg) {
		static pidint_t integral = 0;
		
		pidint_t diff = trg - now;
		
		integral += diff;
		if ( this->has_limit ) {
			if ( integral < -this->integral_max ) { integral = -this->integral_max; }
			if ( integral >  this->integral_max ) { integral =  this->integral_max; }
		}

		return this->kp * diff + this->ki * integral;
	}
};

#endif /* PID_CONTROLLER_H */