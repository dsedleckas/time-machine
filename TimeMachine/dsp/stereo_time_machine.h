#include "time_machine.h"
#include "daisysp.h"

#ifndef STEREO_TIME_MACHINE_H_
#define STEREO_TIME_MACHINE_H_

class StereoTimeMachine {
    public:
        TimeMachine timeMachineLeft;
        TimeMachine timeMachineRight;
        float outputs[2];
        void Init(float sampleRate, float maxDelay, float* bufferLeft, float* bufferRight) {
            timeMachineLeft.Init(sampleRate, maxDelay, bufferLeft);
            timeMachineRight.Init(sampleRate, maxDelay, bufferRight);
        }
        void Set(float dryAmp, float feedback, float blur=0.0) {
            timeMachineLeft.Set(dryAmp, feedback, blur);
            timeMachineRight.Set(dryAmp, feedback, blur);
        }
        float* Process(float inLeft, float inRight) {
            outputs[0] = timeMachineLeft.Process(inLeft);
            outputs[1] = timeMachineRight.Process(inRight);
            return outputs;
        }

        // 0 is DRY
        // 1-8 is ReadHeads
        float GetLoudness(int idx) {
            float loudness = timeMachineLeft.GetLoudness(idx);
            return std::max(loudness, timeMachineRight.GetLoudness(idx));
        }
    };
#endif // STEREO_TIME_MACHINE_H_
