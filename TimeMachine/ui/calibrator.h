#include "../time_machine_hardware.h"
#include "calibration_data.h"

#include "daisy.h"

#include "leds.h"
#include "../dsp/util.h"

#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

class Calibrator {
    public:
        void Init(PersistentStorage<CalibrationData> cs) {
            calibration_storage_ = &cs;

            // load calibration data, using sensible defaults
            calibration_storage_->Init({0.0f, 0.0f, 0.0f, false});
            saved_calibration_data_ = &calibration_storage_->GetSettings();
        }

        void Calibrate(
            oam::time_machine::TimeMachineHardware hw,
            oam::time_machine::Leds leds) {
            if(ShouldCalibrate(hw, leds)) {
		
                // perform calibration routine
                int numSamples = 128;
                for(int i = 0; i < numSamples; i++) {
                    
                    // // accumulate cv values TODO: properly inject new values
                    // saved_calibration_data_->timeCvOffset += timeCv;
                    // saved_calibration_data_->skewCvOffset += skewCv;
                    // saved_calibration_data_->feedbackCvOffset += feedbackCv;

                    // saved_calibration_data_->vca1CvOffset += vca1Cv;
                    // saved_calibration_data_->vca2CvOffset += vca2Cv;
                    // saved_calibration_data_->vca3CvOffset += vca3Cv;
                    // saved_calibration_data_->vca4CvOffset += vca4Cv;

                    // wait 10ms
                    System::Delay(10);

                    // set LEDs
                    for(int ledIndex=0; ledIndex < 9; ledIndex++) {
                        leds.Set(ledIndex, i % 8 < 4 ? 1.0f : 0.0f);
                    }
                }
                
                // divide CVs by number of samples taken to get average
                saved_calibration_data_->timeCvOffset = saved_calibration_data_->timeCvOffset / ((float)numSamples);
                saved_calibration_data_->skewCvOffset = saved_calibration_data_->skewCvOffset / ((float)numSamples);
                saved_calibration_data_->feedbackCvOffset = saved_calibration_data_->feedbackCvOffset / ((float)numSamples);

                saved_calibration_data_->vca1CvOffset = saved_calibration_data_->vca1CvOffset / ((float)numSamples);
                saved_calibration_data_->vca2CvOffset = saved_calibration_data_->vca2CvOffset / ((float)numSamples);
                saved_calibration_data_->vca3CvOffset = saved_calibration_data_->vca3CvOffset / ((float)numSamples);
                saved_calibration_data_->vca4CvOffset = saved_calibration_data_->vca4CvOffset / ((float)numSamples);
                
                // set calibrated value to true
                saved_calibration_data_->calibrated = true;
                
                // save calibration data
                calibration_storage_->Save();
            }
        }

    private:
        PersistentStorage<CalibrationData>* calibration_storage_;

        CalibrationData* saved_calibration_data_;

        bool ShouldCalibrate(
            oam::time_machine::TimeMachineHardware hw,
            oam::time_machine::Leds leds) {
            
            if (!CheckIfInCalibrationPositions(hw)) return false;

            bool calibrationReady = true;
            // do reverse LED startup sequence while
            // checking that we definitely want to calibrate
            int delayBetweenSequencesMs = 100;
            for(int i=0; i < (5000/delayBetweenSequencesMs); i++) {

                leds.InitCalibrationSequence(i);

                System::Delay(delayBetweenSequencesMs);
                
                calibrationReady &= CheckIfInCalibrationPositions(hw);
                if(!calibrationReady) break;
            }

            return calibrationReady;
        }

        bool CheckIfInCalibrationPositions(oam::time_machine::TimeMachineHardware hw) {
            bool shouldCalibrate = \
                (hw.GetAdcValue(SKEW_CV) < 0.01) && \
                (hw.GetAdcValue(TIME_CV) < 0.01) && \
                (hw.GetAdcValue(FEEDBACK_CV) < 0.01) && \
                (hw.GetAdcValue(VCA_1_CV) < 0.01) && \
                (hw.GetAdcValue(VCA_2_CV) < 0.01) && \
                (hw.GetAdcValue(VCA_3_CV) < 0.01) && \
                (hw.GetAdcValue(VCA_4_CV) < 0.01) && \
                hw.gate_in_2.State();
            
            for(int i=0; i<9; i++) {
                shouldCalibrate &= hw.GetSliderValue(i) < 0.01;
            }

            shouldCalibrate &= minMaxKnob(1.0 - hw.GetAdcValue(TIME_KNOB)) > 0.95;
            shouldCalibrate &= minMaxKnob(1.0 - hw.GetAdcValue(SKEW_KNOB)) > 0.95;
            shouldCalibrate &= minMaxKnob(1.0 - hw.GetAdcValue(FEEDBACK_KNOB)) > 0.95;
            return shouldCalibrate;
        }
};

#endif // CALIBRATOR_H_