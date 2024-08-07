#include "daisysp.h"
#include "slew.h"

#ifndef LOUDNESS_DETECTOR_H_
#define LOUDNESS_DETECTOR_H_

class LoudnessDetector {
public:
    Slew slew;
    float lastVal = 0;
    void Init() { slew.Init(); }
    float Get() { return this->lastVal; }
    float Process(float x) {
        lastVal = slew.Process(abs(x));
        return x;
    }
};

#endif // LOUDNESS_DETECTOR_H_