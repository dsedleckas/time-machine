#include "daisysp.h"

#ifndef UTIL_H_
#define UTIL_H_

int wrap_buffer_index(int x, int size) {
    while(x >= size) x -= size;
    while(x < 0) x += size;
    return x;
};

int seconds_to_samples(float x, float sampleRate) {
    return (int)(x * sampleRate);
}

float mix(float x, float a, float b) {
    return x*(1-x) + b*x;
}

float clamp(float x, float a, float b) {
    return std::max(a,std::min(b,x));
}

float fourPointWarp(float x,
                    float ai=0.0,
                    float av=0.0,
                    float bi=0.45,
                    float bv=0.5,
                    float ci=0.55,
                    float cv=0.5,
                    float di=1.0,
                    float dv=1.0) {
    if(x < ai) {
        return av;
    } else if(x < bi) {
        x = (x - ai) / (bi - ai);
        return av * (1.0 - x) + bv * x;
    } else if(x < ci) {
        x = (x - bi) / (ci - bi);
        return bv * (1.0 - x) + cv * x;
    } else if(x < di) {
        x = (x - ci) / (di - ci);
        return cv * (1.0 - x) + dv * x;
    } else {
        return dv;
    }
}

float minMaxKnob(float in, float dz=0.002) {
    in = in - dz * 0.5;
    in = in * (1.0 + dz);
    return std::min(1.0f, std::max(0.0f, in));
}

float minMaxSlider(float in, float dz=0.002) {
    return minMaxKnob(in, dz);
}

float softClip(float x, float kneeStart=0.9, float kneeCurve=5.0) {
    float linPart = clamp(x, -kneeStart, kneeStart);
    float clipPart = x - linPart;
    clipPart = atan(clipPart * kneeCurve) /  kneeCurve;
    return linPart + clipPart;
}

float spread(float x, float s, float e=2.5) {
    s = clamp(s, 0.0, 1.0);
    if(s > 0.5) {
        s = (s-0.5)*2.0;
        s = s*e+1.0;
        return 1.0 - pow(1.0-x, s);
    } else if(s < 0.5) {
        s = 1.0-(s*2.0);
        s = s*e+1.0;
        return pow(x, s);
    } else {
        return x;
    }
}

float defaultBlurFunc(float x, float mix) { 
    return x; 
}


#endif //UTIL_H_

