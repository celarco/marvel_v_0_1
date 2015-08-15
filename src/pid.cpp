#include <../include/pid.h>

void pid::set_coeff(float kp, float kd) {
    gain_kp = kp;
    gain_kd = kd;
}

float pid::loop_once(float err, float err_dot) {
    float command = gain_kp * err + gain_kd * err_dot;
    return command;
}
