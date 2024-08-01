#include "daisysp.h"

#ifndef PRECISE_SLEW_H_
#define PRECISE_SLEW_H_

class PreciseSlew
{
  public:
    PreciseSlew() {}
    ~PreciseSlew() {}
    void Init(float sample_rate, float htime) {
        lastVal  = 0;
        prvhtim_ = -100.0;
        htime_   = htime;

        sample_rate_ = sample_rate;
        onedsr_      = 1.0 / sample_rate_;
    }
    float Process(float in) {
        if(prvhtim_ != htime_)
        {
            c2_      = pow(0.5, onedsr_ / htime_);
            c1_      = 1.0 - c2_;
            prvhtim_ = htime_;
        }
        return lastVal = c1_ * in + c2_ * lastVal;
    }
    inline void SetHtime(float htime) { htime_ = htime; }
    inline float GetHtime() { return htime_; }
    float htime_;
    float c1_, c2_, lastVal, prvhtim_;
    float sample_rate_, onedsr_;
};

#endif // PRECISE_SLEW_H_