#ifndef PID_HEADER
#define PID_HEADER

#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

class pid {
private:
    float gain_kp, gain_kd, gain_ki;
	double err_sum, max_err_sum, dt;
	struct timespec now;
	struct timespec end;
	void calc_dt();
public:
    pid();
	void set_coeff(float kp, float ki, float kd);
    float loop_once(float err, float err_dot);
	void reset_integrator();
	void set_max_sum(double max);
};

#endif
