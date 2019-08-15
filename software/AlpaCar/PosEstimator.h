#ifndef POS_ESTIMATOR_H
#define POS_ESTIMATOR_H

#include <inttypes.h>
#include <math.h>

template <class _Tp = double>
class PosEstimator {
private:
	// length between tires
	_Tp D;
	// 1/D
	double D_inv;
	// threshold angle(rad) between curve and straight trajectry
	double threshold;

	double inv_M_PI = 1.0 / M_PI;

public:
	// position and angle
	double x     = 0.0;
	double y     = 0.0;
	double angle = 0.0;
	double route = 0.0;
	// 
	double scale = 1.0;

	PosEstimator(_Tp length_between_tires, double threshold_angle_rad = 0.08) {
		this->D = length_between_tires;
		this->D_inv = 1.0 / this->D;
		this->threshold = threshold_angle_rad;
	}

	void set_params(_Tp length_between_tires, double threshold_angle_rad = 0.08) {
		this->D = length_between_tires;
		this->D_inv = 1.0 / this->D;
		this->threshold = threshold_angle_rad;
	}

	void set_scale(double val) {
		this->scale = val;
	}

	void update(_Tp del_R, _Tp del_L) {
		// walked length
		double del_Len = (del_R + del_L) * 0.5;
		// rotate angle
		double del_Ang = (del_R - del_L) * this->D_inv;
		// rotate angle divided by 2
		double del_Ang_div2 = del_Ang * 0.5;

		if ( abs(del_Ang) > this->threshold ) {
			// curve trajectry
			del_Len = del_Len / del_Ang_div2 * sin(del_Ang_div2);
		} else {
			// nearly or completely straight trajectry
			// do nothing
		}

		// update position and angle
		this->angle += del_Ang_div2;
		this->x     -= del_Len * sin(this->angle);
		this->y     += del_Len * cos(this->angle);
		this->angle += del_Ang_div2;
		this->route += abs(del_Len);
	}



	double get_angle_deg() {
		return this->angle * 180.0 * inv_M_PI;
	}
	double get_angle_rad() {
		return this->angle;
	}

	double get_true_x() {
		return this->x * this->scale;		
	}
	double get_true_y() {
		return this->y * this->scale;		
	}
	double get_true_route() {
		return this->route * this->scale;		
	}
};

#endif /* POS_ESTIMATOR_H */