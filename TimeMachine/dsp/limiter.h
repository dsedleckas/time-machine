#include "daisysp.h"

#ifndef LIMITER_H_
#define LIMITER_H_
namespace oam {

    class Limiter {
        public:
            float gainCoef;
            float attackCoef;
            float releaseCoef;
            void Init(float sampleRate) {
                gainCoef = 1;
                releaseCoef = 16.0 / sampleRate;
            }
            float Process(float in) {
                float targetGainCoef = 1.0 / std::max(abs(in), (float)1.0);
                if(targetGainCoef < gainCoef) {
                    gainCoef = targetGainCoef;
                } else {
                    gainCoef = gainCoef * (1.0 - releaseCoef) + targetGainCoef*releaseCoef;
                }
                return in * gainCoef;
            }
    };
}


#endif // LIMITER_H_