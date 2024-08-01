#include "daisysp.h"

#ifndef SLEW_H_
#define SLEW_H_

class Slew {
public:
    double lastVal = 0.0;
    double coef = 0.001;
    double noiseFloor = 0.0;
    double noiseCoef = 0.0;
    int settleSamples = 0;
    int settleSamplesThreshold = 96;
    void Init(double coef = 0.001, double nf=0.0) {
        this->coef = coef;
        this->noiseFloor = nf;
    }
    float Process(float x) {
        double c = coef;
        double d = (x - lastVal);
        // if we've set a noise floor
        if(noiseFloor > 0.0) {
            // if we're under the noise floor
            if(abs(d) < noiseFloor) {
                // if the input needs to settle
                if(settleSamples < settleSamplesThreshold) {
                    // keep sampling
                    settleSamples++;
                // if the input is done settling
                } else {
                    // don't change the value
                    d = 0.0;
                }
            // if we're over the noise floor
            } else {
                // reset the settle wait
                settleSamples = 0;
            }
        }
        lastVal = lastVal + d * c;
        return lastVal;
    }
};

#endif // SLEW_H_