#include "daisy.h"
#include "../time_machine_hardware.h"
#include "../dsp/util.h"
#include "../dsp/slew.h"
#include "../dsp/continuous_schmidt.h"
#include "../dsp/clock_rate_detector.h"


#ifndef UI_H_
#define UI_H_

class Ui {
    public:

        ClockRateDetector clockRateDetector;
        ContSchmidt timeKnobSchmidt;
        ContSchmidt timeCvSchmidt;

        Slew timeKnobSlew;
        Slew feedbackKnobSlew;
        Slew distributionKnobSlew;
        Slew timeCvSlew;
        Slew feedbackCvSlew;
        Slew distributionCvSlew;

        Slew vca1CvSlew;
        Slew vca2CvSlew;
        Slew vca3CvSlew;
        Slew vca4CvSlew;

        // global storage for CV/knobs so we don't get them twice to print diagnostics
        float timeCv = 0.0;
        float feedbackCv = 0.0;
        float skewCv = 0.0;

        float vca1Cv = 0.0;
        float vca2Cv = 0.0;
        float vca3Cv = 0.0;
        float vca4Cv = 0.0;

        float timeKnob = 0.0;
        float feedbackKnob = 0.0;
        float skewKnob = 0.0;
        float drySlider = 0.0;
        float delaySliders = 0.0;
        float sliderAmpValues_[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

        float distribution;
        float feedback;
        float time;

        void Init(oam::time_machine::TimeMachineHardware hw) {
            hw_ = &hw;

            // init slew limiter for time (we should tune this more delibrately)
            timeKnobSlew.Init(0.5, 0.0005);
            feedbackKnobSlew.Init(0.5, 0.0005);
            distributionKnobSlew.Init(0.5, 0.0005);
            
            timeCvSlew.Init(0.5, 0.0005);
            feedbackCvSlew.Init(0.5, 0.0005);
            distributionCvSlew.Init(0.5, 0.0005);

            vca1CvSlew.Init(0.5, 0.0005);
            vca2CvSlew.Init(0.5, 0.0005);
            vca3CvSlew.Init(0.5, 0.0005);
            vca4CvSlew.Init(0.5, 0.0005);

            clockRateDetector.Init(hw_->AudioSampleRate());
               
            ProcessAllControls();
        }

        void ProcessClockRate(bool triggered) {
            clockRateDetector.Process(triggered);
        }

        void ProcessAllControls() {
            // process controls
            hw_->ProcessAllControls();

            // populate/update global CV/knob vars (time is slewed to reduce noise at large time values)
            timeKnob = minMaxKnob(1.0 - hw_->GetAdcValue(TIME_KNOB), 0.0008);
            feedbackKnob = fourPointWarp(1.0 - minMaxKnob(hw_->GetAdcValue(FEEDBACK_KNOB), 0.028));
            skewKnob = fourPointWarp(1.0 - minMaxKnob(hw_->GetAdcValue(SKEW_KNOB), 0.0008));

            timeCv = clamp(hw_->GetAdcValue(TIME_CV) - timeCvOffset, -1, 1);
            feedbackCv = clamp(hw_->GetAdcValue(FEEDBACK_CV) - feedbackCvOffset, -1, 1);
            skewCv = clamp(hw_->GetAdcValue(SKEW_CV) - skewCvOffset, -1, 1);

            // read modulation / normalized channels 
            for (int i = 0; i < kNumNormalizedChannels; i++) {
                modulation_values_[i] = 
                    clamp(
                        hw_->GetAdcValue(normalized_channels_[i]) - normalized_offsets_[i],
                        -1, 
                        1);
            }
            
            // read slider values
            drySlider = minMaxSlider(1.0 - hw_->GetAdcValue(DRY_SLIDER));
            for (int i = 1; i < 9; i++) {
                sliderAmpValues_[i-1] = std::max(0.0f, minMaxSlider(1.0f - hw_->GetSliderValue(i)));
            }

            // calculate time based on clock if present, otherwise simple time
            float tmp_time = 0.0; 
            if(clockRateDetector.GetInterval() > 0.0) {
                // 12 quantized steps for knob, 10 for CV (idk what these quanta should actually be)
                // time doubles and halves with each step, they are additive/subtractive
                float timeCoef = pow(2.0, (timeKnobSchmidt.Process((1.0-timeKnob)*12)) + (timeCvSchmidt.Process(timeCv*10))) / pow(2.0, 6.0);
                tmp_time = clockRateDetector.GetInterval() / timeCoef;
                // make sure time is a power of two less than the max time available in the buffer
                while(tmp_time > TIME_SECONDS) tmp_time *= 0.5;
            } else {
                // time linear with knob, scaled v/oct style with CV
                tmp_time = pow(timeKnobSlew.Process(timeKnob), 2.0) * 8.0 / pow(2.0, timeCvSlew.Process(timeCv) * 5.0);
            }

            // force time down to a max value (taking whichever is lesser, the max or the time)
            time = std::min((float)TIME_SECONDS, tmp_time);

            // condition distribution knob value to have deadzone in the middle, add CV
	        distribution = fourPointWarp(distributionKnobSlew.Process(skewKnob)) + distributionCvSlew.Process(skewCv);

            // condition feedback knob to have deadzone in the middle, add CV
	        feedback = clamp(fourPointWarp(feedbackKnobSlew.Process(feedbackKnob)) * 2.0 + feedbackCvSlew.Process(feedbackCv), 0, 3);
        }
    
    private:
        oam::time_machine::TimeMachineHardware* hw_
};


#endif // UI_H_