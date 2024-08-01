#include "read_head.h"
#include "loudness_detector.h"
#include "slew.h"
#include "ultra_slow_dc_blocker.h"
#include "limiter.h"

#ifndef TIME_MACHINE_H_
#define TIME_MACHINE_H_

class TimeMachine {
public:
    ReadHead readHeads[8];
    LoudnessDetector loudness;
	float sampleRate;
    float* buffer;
    int bufferSize;
    int writeHeadPosition;
    float dryAmp;
    float feedback;
    float blur;
    Slew dryAmpSlew;
    Slew feedbackSlew;
    Slew ampCoefSlew;
    Slew blurSlew;
    oam::Limiter outputLimiter;
    oam::Limiter feedbackLimiter;
    UltraSlowDCBlocker dcblk;
    daisysp::Compressor compressor;
    float (*blurFunc)(float, float) = nullptr;
    void Init(float sampleRate, float maxDelay, float* buffer) {
		this->sampleRate = sampleRate;
        this->bufferSize = seconds_to_samples(maxDelay, sampleRate);
        this->buffer = buffer;
        for(int i=0; i<bufferSize; i++) buffer[i] = 0;
        for(int i=0; i<8; i++) readHeads[i].Init(sampleRate, buffer, bufferSize);
        writeHeadPosition = 0;
        dryAmpSlew.Init();
        feedbackSlew.Init(0.01);
        ampCoefSlew.Init(0.0001);
        blurSlew.Init();
        outputLimiter.Init(sampleRate);
        feedbackLimiter.Init(sampleRate);
        blurFunc = defaultBlurFunc;
        loudness.Init();
        compressor.Init(sampleRate);
        compressor.SetAttack(0.02);
        compressor.SetRelease(0.2);
        compressor.SetRatio(5.0);
        compressor.SetThreshold(0.0);
    }
    void Set(float dryAmp, float feedback, float blur=0.0) {
        this->dryAmp = dryAmp;
        this->feedback = feedback;
        this->blur = blur;
    }
    void SetBlurFunc(float (*f)(float, float)) {
        blurFunc = f;
    }
    float Process(float in) {
        float out = 0;

        float ampCoef = 0.0;
        for(int i=0; i<8; i++) ampCoef += readHeads[i].targetAmp;
        ampCoef = ampCoefSlew.Process(1.0 / std::max(1.0f, ampCoef));

        buffer[writeHeadPosition] = loudness.Process(in);
        
        for(int i=0; i<8; i++) out += readHeads[i].Process(writeHeadPosition);
        out = dcblk.Process(out);
        out = compressor.Process(out, buffer[writeHeadPosition] + out);

        buffer[writeHeadPosition] = -(feedbackLimiter.Process(buffer[writeHeadPosition] + (out * feedbackSlew.Process(feedback) * ampCoef)));
        out = outputLimiter.Process(out + in * dryAmpSlew.Process(dryAmp));
        writeHeadPosition = wrap_buffer_index(writeHeadPosition + 1, bufferSize);

        return out;
    }
};

#endif // TIME_MACHINE_H_