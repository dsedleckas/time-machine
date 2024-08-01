#include "daisysp.h"
#include "slew.h"

#ifndef ULTRA_SLOW_DC_BLOCKER_H_
#define ULTRA_SLOW_DC_BLOCKER_H_

class UltraSlowDCBlocker {
public:
    Slew slew;
    void Init(float coef = 0.00001) {
        slew.Init(coef);
    }
    float Process(float x) {
        return x - slew.Process(x);
    }
};

#endif // ULTRA_SLOW_DC_BLOCKER_H_