#include "daisysp.h"

#ifndef CONT_SCHMIDT_H_
#define CONT_SCHMIDT_H_

class ContSchmidt {
public:
    float val = 0.0;
    float Process(float x, float h=0.333) {
        float i, f;
        float sign = x < 0.0 ? -1.0 : 1.0;
        x = abs(x);
	    f = modf(x, &i);
        if(f < h) val = i;
        if(f > 1.0 - h) val = i + 1;
        return val * sign;
    }
};

#endif // CONT_SCHMIDT_H_