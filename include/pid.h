#ifndef PID_HEADER
#define PID_HEADER

class pid {
private:
    float gain_kp,gain_kd;
public:
    void set_coeff(float kp, float kd);
    float loop_once(float err, float err_dot);
};

#endif
