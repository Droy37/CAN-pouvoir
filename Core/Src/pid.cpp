#include "pid.h"

#include "motor.h"

extern M3508_Motor motor;

PID::PID(float kp, float ki, float kd, float i_max, float out_max) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    i_max_ = i_max;
    out_max_ = out_max;
    ref_ = 80;
};

float PID::calc(float ref, float fdb) {
    last_err_ = err_;
    err_ = ref - fdb;
    err_sum_ += err_;

    pout_ = kp_ * err_ * -motor.ratio_;
    iout_ = ki_ * err_sum_ * -motor.ratio_;
    if (iout_ > i_max_) {
        iout_ = i_max_;
    } else if (iout_ < -i_max_) {
        iout_ = -i_max_;
    }

    dout_ = kd_ * (last_err_ - err_);
    output_ = pout_ + iout_+ dout_ ;

    if (output_ > out_max_) {
        output_ = out_max_;
    } else if (output_ < -out_max_) {
        output_ = -out_max_;
    }

    return output_;
}
PID pid(0.9, 0.01, 0, 1500, 3000);
