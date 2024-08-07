#include "daisysp.h"
#include "daisy.h"
#include "util.h"
#include "loudness_detector.h"

#ifndef READ_HEAD_H_
#define READ_HEAD_H_

using namespace daisysp;
using namespace daisy;

class ReadHead {
public:
    LoudnessDetector loudness;
    float* buffer;
    int bufferSize;
    float delayA = 0.0;
    float delayB = 0.0;
    float targetDelay = -1;
    float ampA = 0.0;
    float ampB = 0.0;
    float targetAmp = -1;
	float sampleRate;
    float phase = 1.0;
    float delta;
    float blurAmount;
    void Init(float sampleRate, float* buffer, int bufferSize) {
        this->sampleRate = sampleRate;
        this->delta = 5.0 / sampleRate;
        this->buffer = buffer;
        this->bufferSize = bufferSize;
        this->blurAmount = 0.0;
    }
    void Set(float delay, float amp, float blur = 0) {
        this->targetDelay = delay;
        this->targetAmp = amp;
        this->blurAmount = blur;
    }
    float Process(float writeHeadPosition) {
        if(phase >= 1.0 && (targetDelay >= 0.0 || targetAmp > 0.0)) {
            delayA = delayB;
            delayB = targetDelay;
            targetDelay = -1.0;
            ampA = ampB;
            ampB = targetAmp;
            targetAmp = -1.0;
            phase = 0.0;
            delta = (5.0 + daisy::Random::GetFloat(-blurAmount, blurAmount)) / sampleRate;
        }
        float outputA = this->buffer[wrap_buffer_index(writeHeadPosition - seconds_to_samples(this->delayA, this->sampleRate), bufferSize)];
        float outputB = this->buffer[wrap_buffer_index(writeHeadPosition - seconds_to_samples(this->delayB, this->sampleRate), bufferSize)];
        float output = ((1 - phase) * outputA) + (phase * outputB);
        float outputAmp = ((1 - phase) * ampA) + (phase * ampB);
        phase = phase <= 1.0 ? phase + delta : 1.0;
        return loudness.Process(output) * outputAmp;
    }
};

#endif // READ_HEAD_H_